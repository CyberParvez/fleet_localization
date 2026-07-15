from pathlib import Path
import yaml
import pytest

from fleet_localization.map_catalog import MapCatalogError
from fleet_localization.map_transaction import MapTransaction


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
