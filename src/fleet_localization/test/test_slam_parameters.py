from pathlib import Path
import yaml


def test_fresh_async_mapping_profile_and_launch_injection():
    root=Path(__file__).parents[1]
    profile=yaml.safe_load((root/'config'/'burger_slam_async.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert profile['mode'] == 'mapping'
    assert profile['use_map_saver'] is True
    forbidden=('map_file_name','map_start_pose','map_start_at_dock','deserialize','lifelong','merge')
    assert not any(key in profile for key in forbidden)
    launch=(root/'launch'/'localization.launch.py').read_text()
    assert "executable='async_slam_toolbox_node'" in launch
    assert "'use_lifecycle_manager':True" in launch
    assert "remappings=[('/map','map'),('/map_metadata','map_metadata')]" in launch
    for value in ("interface.frame('map')","interface.frame('odom')","interface.frame('base_footprint')","'scan_topic':'scan'"):
        assert value in launch
