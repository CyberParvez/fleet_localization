from fleet_localization.operator_config import render_rviz
from pathlib import Path


def test_robot_only_rviz_render(tmp_path):
    template=Path(__file__).parents[1]/'rviz'/'robot_localization.rviz'
    output=render_rviz(template,'robot1','/robot1','robot1')
    try: text=output.read_text()
    finally: output.unlink()
    for expected in ('Fixed Frame: robot1/map','/robot1/map','/robot1/scan','/robot1/odometry/filtered','/robot1/amcl_pose','Topic: /robot1/initialpose'):
        assert expected in text
    for forbidden in ('Navigate','Goal','Costmap','/robot2','@ROBOT@','@NS@','@PREFIX@'):
        assert forbidden not in text
    assert 'All Enabled: false' in text
