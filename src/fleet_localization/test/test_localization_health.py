from fleet_localization.validation import LocalizationHealthState, Persistence


def healthy_state():
    return LocalizationHealthState(persistence=Persistence(2.0,2.0))


def test_awaits_manual_pose_then_requires_post_initialization_estimate():
    state=healthy_state()
    assert state.evaluate(10.0,[],True,True,True)[0]=='awaiting initial pose'
    state.initialized(10.0)
    assert state.evaluate(10.1,[],True,True,True)[0]=='localizing'
    state.estimate(10.2,[0.1 if i in (0,7,35) else 0.0 for i in range(36)])
    assert state.evaluate(10.2,[],True,True,True)==('localized',[])


def test_preinitialization_sensor_failure_can_degrade():
    state=healthy_state()
    assert state.evaluate(1.0,['scan: stale'],True,True,False)[0]=='awaiting initial pose'
    assert state.evaluate(3.0,['scan: stale'],True,True,False)[0]=='degraded'


def test_covariance_boundaries_and_freshness():
    state=healthy_state(); state.initialized(1.0)
    covariance=[0.0]*36; covariance[0]=covariance[7]=0.25; covariance[35]=0.10
    state.estimate(1.0,covariance)
    assert state.evaluate(2.0,[],True,True,True)[0]=='localized'
    covariance[0]=0.251; state.estimate(2.0,covariance)
    assert 'position covariance' in ' '.join(state.evaluate(2.0,[],True,True,True)[1])
    covariance[0]=0.1; covariance[35]=0.101; state.estimate(2.1,covariance)
    assert 'yaw covariance' in ' '.join(state.evaluate(2.1,[],True,True,True)[1])


def test_two_second_degradation_and_recovery_persistence():
    state=healthy_state(); state.initialized(1.0); covariance=[0.0]*36
    covariance[0]=covariance[7]=covariance[35]=0.1; state.estimate(1.0,covariance)
    assert state.evaluate(1.0,['scan: stale'],True,True,True)[0]=='localizing'
    assert state.evaluate(3.0,['scan: stale'],True,True,True)[0]=='degraded'
    state.estimate(3.1,covariance)
    assert state.evaluate(3.1,[],True,True,True)[0]=='degraded'
    state.estimate(5.1,covariance)
    assert state.evaluate(5.1,[],True,True,True)[0]=='localized'


def test_broken_lifecycle_map_and_tf_are_health_violations():
    state=healthy_state(); state.initialized(1.0); covariance=[0.0]*36
    state.estimate(1.0,covariance)
    _,problems=state.evaluate(1.0,[],False,False,False)
    assert problems==['localization lifecycle inactive','map unavailable','map-to-sensor transform unavailable']
