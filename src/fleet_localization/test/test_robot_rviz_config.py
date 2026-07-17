from fleet_localization.operator_config import render_rviz
from pathlib import Path


def test_robot_only_rviz_render(tmp_path):
    template=Path(__file__).parents[1]/'rviz'/'robot_localization.rviz'
    output=render_rviz(template,'robot1','/robot1','robot1')
    try: text=output.read_text()
    finally: output.unlink()
    for expected in ('Fixed Frame: robot1/map','/robot1/map','/robot1/scan','/robot1/amcl_pose','Topic: /robot1/initialpose'):
        assert expected in text
    for forbidden in ('Navigate','Goal','Costmap','/robot2','@ROBOT@','@NS@','@PREFIX@'):
        assert forbidden not in text
    assert 'All Enabled: false' in text
    assert 'rviz_default_plugins/Odometry' not in text
    assert '/robot1/odometry/filtered' not in text
    for display in ('rviz_default_plugins/Grid','rviz_default_plugins/Map',
                    'rviz_default_plugins/LaserScan','rviz_default_plugins/RobotModel',
                    'rviz_default_plugins/PoseWithCovariance','rviz_default_plugins/TF'):
        assert display in text
    assert 'Class: rviz_default_plugins/Orbit' in text
    assert 'Target Frame: robot1/map' in text
    assert text.count('Enabled: true') >= 7
