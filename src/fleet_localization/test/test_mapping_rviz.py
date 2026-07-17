from pathlib import Path


ROOT=Path(__file__).parents[1]


def test_mapping_uses_dedicated_robot_scoped_view():
    launch=(ROOT/'launch'/'localization.launch.py').read_text()
    assert "rviz_template='robot_localization.rviz' if mode == 'localization' else 'robot_mapping.rviz'" in launch
    view=(ROOT/'rviz'/'robot_mapping.rviz').read_text()
    for topic in ('@NS@/map','@NS@/scan','@NS@/robot_description'):
        assert topic in view
    for frame in ('@PREFIX@/map','@PREFIX@/odom','@PREFIX@/base_footprint','@PREFIX@/base_scan'):
        assert frame in view
    for display in ('rviz_default_plugins/Grid','rviz_default_plugins/Map',
                    'rviz_default_plugins/LaserScan','rviz_default_plugins/RobotModel',
                    'rviz_default_plugins/TF'):
        assert display in view
    assert 'Fixed Frame: @PREFIX@/map' in view
    assert 'Class: rviz_default_plugins/Orbit' in view
    assert 'Target Frame: @PREFIX@/map' in view
    assert view.count('Enabled: true') >= 7
    forbidden=('AMCL','amcl_pose','initialpose','SetInitialPose','Goal','Nav2')
    assert not any(value in view for value in forbidden)
    assert 'rviz_default_plugins/Odometry' not in view
    assert '@NS@/odometry/filtered' not in view
    assert 'rviz_default_plugins/Interact' in view
    assert 'rviz_default_plugins/MoveCamera' in view
