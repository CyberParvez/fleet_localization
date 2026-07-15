"""Owner-appropriate localization/mapping fault matrix without production hooks."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from fleet_localization.map_catalog import MapCatalogError, resolve_map
from fleet_localization.map_store_lock import MapStoreBusy, MapStoreLock
from fleet_localization.map_transaction import MapTransaction
from fleet_localization.readiness import Readiness
from fleet_localization.validation import LocalizationHealthState, Persistence


def manifest(tmp_path, *, metadata_world='turtlebot3_world'):
    store = tmp_path / 'maps'; directory = store / 'base'; directory.mkdir(parents=True)
    (directory / 'map.pgm').write_text('P2\n1 1\n255\n0\n')
    (directory / 'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':0.1,
        'origin':[0.0,0.0,0.0],'negate':0,'occupied_thresh':0.65,'free_thresh':0.196}))
    (directory / 'world.yaml').write_text(yaml.safe_dump({'map_id':'base','world':metadata_world,'source':'fault fixture'}))
    path = tmp_path / 'fleet.yaml'; path.write_text(yaml.safe_dump({
        'fleet':{'map_store':'./maps','map_id':'base'},
        'simulation':{'world':'turtlebot3_world','gui':False},
        'robots':[{'name':'robot1','namespace':'/robot1','frame_prefix':'robot1','model':'burger','spawn':{'x':0.0,'y':0.0}}]}))
    return path, store


def stamp(seconds):
    return SimpleNamespace(sec=int(seconds), nanosec=int((seconds-int(seconds))*1e9))


def test_bad_and_stale_timestamp_degrades_only_affected_state():
    node = Readiness.__new__(Readiness); node.sim_now=10.0; node.valid={}; node.reasons={}; node.last_stamp={}
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(0.0), frame_id='robot1/imu_link'))
    node._common('imu', message, 'robot1/imu_link', [1.0])
    assert node.reasons['imu'] == 'zero timestamp'
    first = LocalizationHealthState(persistence=Persistence(2.0,2.0)); second = LocalizationHealthState(persistence=Persistence(2.0,2.0))
    first.evaluate(10.0,['scan: stale'],True,True,True)
    assert first.evaluate(12.0,['scan: stale'],True,True,True)[0] == 'degraded'
    second.initialized(12.0); second.estimate(12.0,[0.05]*36)
    assert second.evaluate(12.0,[],True,True,True)[0] == 'localized'


def test_lifecycle_failure_and_metadata_mismatch_are_explicit(tmp_path):
    state=LocalizationHealthState(persistence=Persistence(0.0,2.0)); state.initialized(1.0)
    status, problems=state.evaluate(1.0,[],False,True,True)
    assert status == 'degraded' and 'localization lifecycle inactive' in problems
    config,_=manifest(tmp_path,metadata_world='asymmetric_indoor')
    with pytest.raises(MapCatalogError,match='active world'): resolve_map(config)


def test_lock_contention_duplicate_save_and_no_implicit_save(tmp_path):
    config,store=manifest(tmp_path)
    with MapStoreLock.acquire_shared(store):
        with pytest.raises(MapStoreBusy): MapStoreLock.acquire_exclusive(store)
    with MapStoreLock.acquire_exclusive(store):
        with MapTransaction(config,'new') as transaction:
            (transaction.staging/'map.pgm').write_text('P2\n1 1\n255\n0\n')
            (transaction.staging/'map.yaml').write_text((store/'base/map.yaml').read_text())
            transaction.commit()
    with pytest.raises(MapCatalogError,match='already exists'):
        with MapTransaction(config,'new'):
            pass
    launch=(Path(__file__).parents[1]/'launch/localization.launch.py').read_text()
    assert 'map_save' not in launch
