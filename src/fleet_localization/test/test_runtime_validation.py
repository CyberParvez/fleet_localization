"""Callback-level regressions for strict runtime input and TF validation."""
from types import SimpleNamespace

from fleet_localization.health import Health
from fleet_localization.readiness import Readiness
from fleet_localization.validation import LocalizationHealthState, MeasurementValidator, Observations, Persistence


def stamp(value):
    return SimpleNamespace(sec=int(value),nanosec=int((value-int(value))*1e9))


class Interface:
    def frame(self,suffix): return f'robot1/{suffix}'


def health(now=10.0):
    node=Health.__new__(Health); node.sim_now=now; node.interface=Interface()
    node.obs=Observations(); node.validator=MeasurementValidator()
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


def test_preexisting_selected_tf_edges_block_readiness_with_actionable_reason():
    node=Readiness.__new__(Readiness); node.interface=Interface(); node.valid={}; node.reasons={}; node.tf_conflict=None
    transform=SimpleNamespace(header=SimpleNamespace(frame_id='robot1/odom'),child_frame_id='robot1/base_footprint')
    node.tf_message(SimpleNamespace(transforms=[transform]))
    assert 'tf_authority' not in node.valid
    assert 'disable the simulator or external estimator broadcaster' in node.reasons['tf_authority']


def test_unrelated_shared_tf_edge_does_not_trigger_conflict():
    node=Readiness.__new__(Readiness); node.interface=Interface(); node.valid={}; node.reasons={}; node.tf_conflict=None
    transform=SimpleNamespace(header=SimpleNamespace(frame_id='robot2/odom'),child_frame_id='robot2/base_footprint')
    node.tf_message(SimpleNamespace(transforms=[transform]))
    assert node.tf_conflict is None
