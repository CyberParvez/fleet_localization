import importlib.util
from pathlib import Path

def test_launch_declares_aligned_interface_and_gate():
    text=(Path(__file__).parents[1]/'launch'/'localization.launch.py').read_text()
    for argument in ('fleet_config','robot','mode','map_id','rviz','timeout'): assert f"DeclareLaunchArgument('{argument}'" in text
    assert 'OnProcessExit' in text and "package='robot_localization'" in text
    assert text.index("executable='readiness'") < text.index("executable='ekf_node'")
    assert "package='nav2_map_server'" in text and "package='nav2_amcl'" in text
    assert "'node_names':['map_server','amcl']" in text and "'autostart':True" in text
    assert 'slam_toolbox' not in text.lower()
