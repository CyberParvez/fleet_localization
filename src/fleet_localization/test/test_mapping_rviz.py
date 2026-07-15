from pathlib import Path


ROOT=Path(__file__).parents[1]


def test_mapping_uses_dedicated_robot_scoped_view():
    launch=(ROOT/'launch'/'localization.launch.py').read_text()
    assert "rviz_template='robot_localization.rviz' if mode == 'localization' else 'robot_mapping.rviz'" in launch
    view=(ROOT/'rviz'/'robot_mapping.rviz').read_text()
    for topic in ('@NS@/map','@NS@/scan','@NS@/odometry/filtered','@NS@/robot_description'):
        assert topic in view
    for frame in ('@PREFIX@/map','@PREFIX@/odom','@PREFIX@/base_footprint','@PREFIX@/base_scan'):
        assert frame in view
    forbidden=('AMCL','amcl_pose','initialpose','SetInitialPose','Goal','Nav2')
    assert not any(value in view for value in forbidden)
    assert 'Tools: []' in view
