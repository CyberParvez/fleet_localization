"""Independent manual-initialization and health-state regressions."""
from fleet_localization.validation import LocalizationHealthState, Persistence


def _state():
    return LocalizationHealthState(persistence=Persistence(2.0, 2.0))


def _covariance():
    values = [0.0] * 36
    values[0] = values[7] = values[35] = 0.05
    return values


def test_initializing_robot1_does_not_initialize_robot2():
    robot1, robot2 = _state(), _state()
    robot1.initialized(10.0)
    robot1.estimate(10.1, _covariance())
    assert robot1.evaluate(10.1, [], True, True, True)[0] == 'localized'
    assert robot2.evaluate(10.1, [], True, True, True)[0] == 'awaiting initial pose'


def test_each_robot_can_degrade_and_recover_independently():
    robot1, robot2 = _state(), _state()
    for state in (robot1, robot2):
        state.initialized(1.0); state.estimate(1.0, _covariance())
    assert robot1.evaluate(1.0, ['scan: stale'], True, True, True)[0] == 'localizing'
    assert robot1.evaluate(3.0, ['scan: stale'], True, True, True)[0] == 'degraded'
    robot2.estimate(3.0, _covariance())
    assert robot2.evaluate(3.0, [], True, True, True)[0] == 'localized'
