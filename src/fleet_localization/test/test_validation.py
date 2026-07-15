from fleet_localization.validation import Observations, Persistence, covariance_valid, finite

def test_numeric_and_covariance_validation():
    assert finite([0.0,1.0]); assert not finite([float('nan')])
    assert covariance_valid([1.0]*36,(0,35)); assert not covariance_valid([0.0]*36,(0,35))

def test_observation_aggregation():
    o=Observations(last={'wheel':10.0},invalid={'imu':'bad covariance'})
    assert o.problems(12.0,('wheel','imu','scan'))==['wheel: stale','imu: bad covariance','scan: missing']

def test_degradation_and_recovery_are_persistent():
    p=Persistence()
    assert p.update(0.0,['missing'])=='waiting for inputs'
    assert p.update(2.0,['missing'])=='degraded'
    assert p.update(3.0,[])=='degraded'
    assert p.update(5.0,[])=='ready'
