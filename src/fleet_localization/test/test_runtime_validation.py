"""Callback-level regressions for strict runtime input and TF validation."""
from types import SimpleNamespace

from fleet_localization.health import Health
from fleet_localization.readiness import Readiness
from fleet_localization.validation import (LocalizationHealthState, MeasurementValidator,
    Observations, Persistence, TfAuthorityTracker, publisher_gid)


def stamp(value):
    return SimpleNamespace(sec=int(value),nanosec=int((value-int(value))*1e9))


class Interface:
    def frame(self,suffix): return f'robot1/{suffix}'


def health(now=10.0):
    node=Health.__new__(Health); node.sim_now=now; node.interface=Interface()
    node.obs=Observations(); node.validator=MeasurementValidator(); node.tf_tracker=TfAuthorityTracker(set())
    node.state=LocalizationHealthState(persistence=Persistence(2.0,2.0))
    node.map_seen=True; node.map_at=now
    return node


def wheel(at=9.9,frame='robot1/odom',child='robot1/base_footprint',variance=0.1):
    covariance=[0.0]*36; covariance[0]=covariance[35]=variance
    twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.1),angular=SimpleNamespace(z=0.0)),covariance=covariance)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(at),frame_id=frame),child_frame_id=child,twist=twist)


def test_preclock_and_bad_frame_samples_do_not_become_health_observations():
    node=health(0.0); node.seen('wheel',wheel())
    assert 'wheel' not in node.obs.last and node.obs.invalid['wheel']=='simulation clock unavailable'
    node.sim_now=10.0; node.seen('wheel',wheel(frame='other/odom'))
    assert 'expected' in node.obs.invalid['wheel']


def test_future_out_of_order_and_invalid_covariance_are_rejected_then_recover():
    node=health(); node.seen('wheel',wheel())
    assert node.obs.last['wheel']==9.9
    node.seen('wheel',wheel(11.0)); assert node.obs.invalid['wheel']=='future timestamp'
    node.seen('wheel',wheel(9.5)); assert node.obs.invalid['wheel']=='out-of-order timestamp'
    node.seen('wheel',wheel(10.0,variance=-0.1)); assert node.obs.invalid['wheel']=='invalid covariance'
    node.seen('wheel',wheel(10.0)); assert node.obs.last['wheel']==10.0 and 'wheel' not in node.obs.invalid


def test_clock_epoch_rollback_clears_timestamp_dependent_health_state():
    node=health(50.0); node.obs.last['wheel']=49.9; node.validator.last_stamp['wheel']=49.9
    node.state.initialized(49.0); node.state.persistence.degraded=True
    node.clock(SimpleNamespace(clock=stamp(2.0)))
    assert node.obs.last=={} and node.validator.last_stamp=={}
    assert node.state.initialized_at is None and not node.state.persistence.degraded
    assert not node.map_seen and node.map_at is None


def test_negative_amcl_required_variance_is_never_localized():
    node=health(); node.state.initialized(9.0)
    covariance=[0.0]*36; covariance[0]=covariance[7]=0.1; covariance[35]=-0.1
    pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0,y=0.0),
        orientation=SimpleNamespace(z=0.0,w=1.0)),covariance=covariance)
    node.amcl_pose(SimpleNamespace(header=SimpleNamespace(stamp=stamp(9.9),frame_id='robot1/map'),pose=pose))
    assert node.state.amcl_at is None and node.obs.invalid['amcl']=='invalid covariance'


def test_zero_stamped_initial_pose_is_rejected_without_initializing_state():
    node=health()
    covariance=[0.0]*36; covariance[0]=covariance[7]=covariance[35]=0.1
    pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0,y=0.0),
        orientation=SimpleNamespace(x=0.0,y=0.0,z=0.0,w=1.0)),covariance=covariance)
    message=SimpleNamespace(header=SimpleNamespace(stamp=stamp(0.0),frame_id='robot1/map'),pose=pose)
    node.initialized(message)
    assert node.state.initialized_at is None
    assert node.obs.invalid['initialpose']=='zero timestamp'


def test_preexisting_selected_tf_edges_block_readiness_with_actionable_reason(monkeypatch):
    node=Readiness.__new__(Readiness); node.interface=Interface(); node.valid={}; node.reasons={}
    node.tf_tracker=TfAuthorityTracker({('robot1/odom','robot1/base_footprint'),('robot1/map','robot1/odom')})
    node.tf_discovery_started=1.0; node.sim_now=10.0; node.started=1.0; node.timeout=60.0
    node.exit_code=None; node.buffer=SimpleNamespace(lookup_transform=lambda *_: object())
    node.get_logger=lambda:SimpleNamespace(info=lambda *_:None,error=lambda *_:None)
    monkeypatch.setattr('fleet_localization.readiness.time.monotonic',lambda:2.0)
    transform=SimpleNamespace(header=SimpleNamespace(frame_id='robot1/odom'),child_frame_id='robot1/base_footprint')
    node.tf_message(SimpleNamespace(transforms=[transform]),SimpleNamespace(publisher_gid=b'publisher-a'))
    node.check()
    assert 'tf_authority' not in node.valid
    assert 'disable the simulator or external estimator broadcaster' in node.reasons['tf_authority']


def test_unrelated_shared_tf_edge_does_not_trigger_conflict():
    node=Readiness.__new__(Readiness); node.interface=Interface(); node.valid={}; node.reasons={}
    node.tf_tracker=TfAuthorityTracker({('robot1/odom','robot1/base_footprint'),('robot1/map','robot1/odom')})
    transform=SimpleNamespace(header=SimpleNamespace(frame_id='robot2/odom'),child_frame_id='robot2/base_footprint')
    node.tf_message(SimpleNamespace(transforms=[transform]),SimpleNamespace(publisher_gid=b'publisher-b'))
    assert node.tf_tracker.observed_edges() == set()


def test_tf_authority_tracker_conflicts_only_for_two_gids_on_same_owned_edge():
    edge=('robot1/odom','robot1/base_footprint'); tracker=TfAuthorityTracker({edge,('robot1/map','robot1/odom')})
    first=SimpleNamespace(header=SimpleNamespace(frame_id=edge[0]),child_frame_id=edge[1])
    other=SimpleNamespace(header=SimpleNamespace(frame_id='robot1/map'),child_frame_id='robot1/odom')
    tracker.observe([first],b'ekf'); tracker.observe([other],b'amcl')
    assert tracker.conflicts() == {}
    tracker.observe([first],b'duplicate-ekf')
    assert set(tracker.conflicts()[edge]) == {b'ekf',b'duplicate-ekf'}


def test_jazzy_message_info_without_gid_keeps_edge_observation_without_false_conflict():
    assert publisher_gid({'publication_sequence_number':1,'received_timestamp':2}) is None
    edge=('robot1/odom','robot1/base_footprint'); tracker=TfAuthorityTracker({edge})
    transform=SimpleNamespace(header=SimpleNamespace(frame_id=edge[0]),child_frame_id=edge[1])
    tracker.observe([transform],None); tracker.observe([transform],None)
    assert tracker.observed_edges()=={edge} and tracker.conflicts()=={}


def test_runtime_tf_conflict_is_persistent_and_actionable():
    node=health(); edge=('robot1/odom','robot1/base_footprint')
    node.tf_tracker=TfAuthorityTracker({edge})
    transform=SimpleNamespace(header=SimpleNamespace(frame_id=edge[0]),child_frame_id=edge[1])
    node.tf_message(SimpleNamespace(transforms=[transform]),SimpleNamespace(publisher_gid=b'ekf'))
    assert node.tf_authority_problems() == []
    node.tf_message(SimpleNamespace(transforms=[transform]),SimpleNamespace(publisher_gid=b'other'))
    assert node.tf_authority_problems() == [
        'duplicate TF authority: robot1/odom -> robot1/base_footprint (2 publishers)']
    node.tf_message(SimpleNamespace(transforms=[transform]),SimpleNamespace(publisher_gid=b'ekf'))
    assert node.tf_authority_problems()  # repeated expected traffic cannot clear the conflict


def occupancy_grid(at=9.9,frame='robot1/map',resolution=0.05,width=2,height=2,data=None):
    origin=SimpleNamespace(position=SimpleNamespace(x=0.0,y=0.0,z=0.0),
        orientation=SimpleNamespace(x=0.0,y=0.0,z=0.0,w=1.0))
    info=SimpleNamespace(resolution=resolution,width=width,height=height,origin=origin)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(at),frame_id=frame),info=info,
        data=[0]*(width*height) if data is None else data)


def test_map_callback_requires_selected_frame_current_stamp_and_valid_grid():
    node=health(); node.map_received(occupancy_grid())
    assert node.map_seen and node.map_at==9.9
    node.map_received(occupancy_grid(11.0)); assert not node.map_seen and node.obs.invalid['map']=='future timestamp'
    node.map_received(occupancy_grid(10.0,frame='map')); assert 'expected' in node.obs.invalid['map']
    node.map_received(occupancy_grid(10.0,resolution=0.0)); assert node.obs.invalid['map']=='invalid resolution'
    node.map_received(occupancy_grid(10.0,data=[0])); assert node.obs.invalid['map']=='invalid dimensions or data length'


def test_observation_freshness_is_configured():
    observations=Observations(freshness=2.5,last={'wheel':8.0})
    assert observations.problems(10.0,('wheel',)) == []
    assert observations.problems(10.6,('wheel',)) == ['wheel: stale']
