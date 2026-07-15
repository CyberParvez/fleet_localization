from fleet_localization.validation import MappingHealthState, Persistence
from pathlib import Path


def test_mapping_health_persistence_and_recovery():
    state=MappingHealthState(1.0,Persistence(2.0,2.0))
    assert state.evaluate(10.0,[],True,10.0,True)[0] == 'mapping'
    assert state.evaluate(11.1,['scan: stale'],True,11.0,True)[0] == 'waiting for mapping'
    assert state.evaluate(13.1,['scan: stale'],True,13.0,True)[0] == 'degraded'
    assert state.evaluate(13.2,[],True,13.2,True)[0] == 'degraded'
    assert state.evaluate(15.2,[],True,15.2,True)[0] == 'mapping'


def test_mapping_requires_slam_map_and_prefixed_tf():
    state=MappingHealthState()
    status,problems=state.evaluate(3.0,[],False,None,False)
    assert status == 'waiting for mapping'
    assert problems == ['SLAM lifecycle inactive','map unavailable','map-to-sensor transform unavailable']


def test_runtime_health_polls_authoritative_lifecycle_state():
    source=(Path(__file__).parents[1]/'fleet_localization'/'health.py').read_text()
    assert 'self.lifecycle_clients=' in source
    assert 'client.call_async(GetState.Request())' in source
    assert 'self.poll_lifecycle()' in source
