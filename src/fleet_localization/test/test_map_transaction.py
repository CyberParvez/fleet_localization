from pathlib import Path
import os
import shutil
import subprocess
import sys
import time
import yaml
import pytest

from fleet_localization.map_catalog import MapCatalogError
from fleet_localization.map_transaction import MapTransaction
from fleet_localization import map_transaction


def manifest(tmp_path):
    store=tmp_path/'maps'; store.mkdir()
    config=tmp_path/'fleet.yaml'
    config.write_text(yaml.safe_dump({'fleet':{'map_store':'./maps','map_id':'old'},
      'simulation':{'world':'turtlebot3_world','gui':False},'robots':[{'name':'robot1','namespace':'/robot1',
      'frame_prefix':'robot1','model':'burger','spawn':{'x':0.0,'y':0.0,'z':0.01,'yaw':0.0}}]}))
    return config,store


def saved(transaction):
    (transaction.staging/'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (transaction.staging/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':0.05,
      'origin':[0.0,0.0,0.0],'negate':0,'occupied_thresh':0.65,'free_thresh':0.196}))


def test_commit_is_complete_valid_and_immutable(tmp_path):
    config,store=manifest(tmp_path)
    with MapTransaction(config,'new_map') as transaction:
        saved(transaction); resolved=transaction.commit()
    assert resolved.map_id == 'new_map'
    assert (store/'new_map'/'world.yaml').is_file()
    with pytest.raises(MapCatalogError):
        with MapTransaction(config,'new_map'): pass


def test_failure_cleans_private_staging_and_publishes_nothing(tmp_path):
    config,store=manifest(tmp_path)
    with pytest.raises(MapCatalogError):
        with MapTransaction(config,'broken') as transaction: transaction.commit()
    assert not (store/'broken').exists()
    assert not list(store.glob('.fleet-map-staging-*'))


def test_unsafe_id_and_image_escape_are_rejected(tmp_path):
    config,store=manifest(tmp_path)
    with pytest.raises(MapCatalogError): MapTransaction(config,'../escape')
    with pytest.raises(MapCatalogError):
        with MapTransaction(config,'escape') as transaction:
            (transaction.staging/'map.yaml').write_text(yaml.safe_dump({'image':'../outside.pgm'}))
            transaction.commit()


@pytest.mark.parametrize('change', [
    {'resolution': 0.0}, {'resolution': float('nan')}, {'origin': [0.0, 0.0]},
    {'origin': [0.0, float('inf'), 0.0]}, {'negate': True},
    {'free_thresh': 0.8}, {'occupied_thresh': float('nan')}, {'mode': 'unknown'},
])
def test_full_occupancy_contract_is_validated_before_publication(tmp_path, change):
    config,store=manifest(tmp_path)
    with pytest.raises(MapCatalogError):
        with MapTransaction(config,'invalid') as transaction:
            saved(transaction)
            data=yaml.safe_load((transaction.staging/'map.yaml').read_text())
            data.update(change)
            (transaction.staging/'map.yaml').write_text(yaml.safe_dump(data))
            transaction.commit()
    assert not (store/'invalid').exists()
    assert not list(store.glob('.fleet-map-staging-*'))


def test_metadata_write_and_validation_failures_never_publish(tmp_path, monkeypatch):
    config,store=manifest(tmp_path)
    monkeypatch.setattr(map_transaction, '_write_world_metadata',
                        lambda *_args: (_ for _ in ()).throw(OSError('disk error')))
    with pytest.raises(OSError, match='disk error'):
        with MapTransaction(config,'metadata_error') as transaction:
            saved(transaction); transaction.commit()
    assert not (store/'metadata_error').exists()
    assert not list(store.glob('.fleet-map-staging-*'))

    monkeypatch.undo()
    monkeypatch.setattr(map_transaction, 'validate_map_directory',
                        lambda *_args: (_ for _ in ()).throw(MapCatalogError('injected validation')))
    with pytest.raises(MapCatalogError, match='injected validation'):
        with MapTransaction(config,'validation_error') as transaction:
            saved(transaction); transaction.commit()
    assert not (store/'validation_error').exists()
    assert not list(store.glob('.fleet-map-staging-*'))


def test_canonical_validation_completes_before_atomic_rename(tmp_path, monkeypatch):
    config,store=manifest(tmp_path)
    events=[]
    original_validate=map_transaction.validate_map_directory
    original_rename=map_transaction._rename_noreplace
    def validate(*arguments):
        assert not (store/'ordered').exists()
        result=original_validate(*arguments)
        events.append(('validated', result.metadata_path.read_text()))
        return result
    def rename(source, destination):
        assert events and events[0][0] == 'validated'
        assert (source/'world.yaml').is_file()
        events.append(('renamed', destination.name))
        original_rename(source, destination)
    monkeypatch.setattr(map_transaction, 'validate_map_directory', validate)
    monkeypatch.setattr(map_transaction, '_rename_noreplace', rename)
    with MapTransaction(config,'ordered') as transaction:
        saved(transaction); transaction.commit()
    assert [event[0] for event in events] == ['validated', 'renamed']
    assert (store/'ordered').is_dir()


def test_late_rename_collision_and_error_preserve_final_and_cleanup(tmp_path, monkeypatch):
    config,store=manifest(tmp_path)
    def collide(_source, destination):
        destination.mkdir()
        (destination/'owner').write_text('other writer')
        raise MapCatalogError("map ID already exists: 'collision'")
    monkeypatch.setattr(map_transaction, '_rename_noreplace', collide)
    with pytest.raises(MapCatalogError, match='already exists'):
        with MapTransaction(config,'collision') as transaction:
            saved(transaction); transaction.commit()
    assert (store/'collision'/'owner').read_text() == 'other writer'
    assert not list(store.glob('.fleet-map-staging-*'))

    monkeypatch.setattr(map_transaction, '_rename_noreplace',
                        lambda *_args: (_ for _ in ()).throw(OSError(5, 'I/O error')))
    with pytest.raises(OSError, match='I/O error'):
        with MapTransaction(config,'rename_error') as transaction:
            saved(transaction); transaction.commit()
    assert not (store/'rename_error').exists()
    assert not list(store.glob('.fleet-map-staging-*'))


def test_subprocess_interruption_during_validation_exposes_no_final_invalid_id(tmp_path):
    config,store=manifest(tmp_path)
    marker=tmp_path/'validating'
    script='''
import pathlib, time, yaml
from fleet_localization import map_transaction as module
from fleet_localization.map_transaction import MapTransaction
config, marker = pathlib.Path(__import__('sys').argv[1]), pathlib.Path(__import__('sys').argv[2])
original = module.validate_map_directory
def paused(*args):
    marker.write_text('validation entered')
    time.sleep(30)
    return original(*args)
module.validate_map_directory = paused
with MapTransaction(config, 'interrupted') as transaction:
    (transaction.staging/'map.pgm').write_bytes(b'P5\\n1 1\\n255\\n\\x00')
    (transaction.staging/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm'}))
    transaction.commit()
'''
    env=os.environ.copy()
    env['PYTHONPATH']=os.pathsep.join(p for p in sys.path if p)
    process=subprocess.Popen([sys.executable,'-c',script,str(config),str(marker)], env=env)
    try:
        deadline=time.monotonic()+10
        while not marker.exists() and process.poll() is None and time.monotonic()<deadline:
            time.sleep(0.02)
        assert marker.exists(), f'subprocess exited early: {process.poll()}'
        assert not (store/'interrupted').exists()
        process.terminate(); process.wait(timeout=5)
        assert not (store/'interrupted').exists()
    finally:
        if process.poll() is None:
            process.kill(); process.wait(timeout=5)
        for staging in store.glob('.fleet-map-staging-*'):
            shutil.rmtree(staging)
