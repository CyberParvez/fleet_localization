from pathlib import Path
import pytest
from fleet_localization.interfaces import resolve_robot
from turtlebot_fleet_sim.fleet_config import FleetConfigError

MANIFEST=Path(__file__).parents[2]/'turtlebot_fleet_sim'/'config'/'fleet.example.yaml'
def test_selected_robot_contract():
    i=resolve_robot(str(MANIFEST),'robot1')
    assert i.wheel_topic=='/robot1/wheel/odometry'
    assert i.filtered_topic=='/robot1/odometry/filtered'
    assert i.frame('odom')=='robot1/odom'
def test_unknown_robot_rejected():
    with pytest.raises(FleetConfigError): resolve_robot(str(MANIFEST),'absent')
