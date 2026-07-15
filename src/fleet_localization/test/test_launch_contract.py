import importlib.util
from pathlib import Path

def test_launch_declares_aligned_interface_and_gate():
    text=(Path(__file__).parents[1]/'launch'/'localization.launch.py').read_text()
    for argument in ('fleet_config','robot','mode','map_id','rviz','timeout'): assert f"DeclareLaunchArgument('{argument}'" in text
    assert 'OnProcessExit' in text and "package='robot_localization'" in text
    assert text.index("executable='readiness'") < text.index("executable='ekf_node'")
    assert 'amcl' not in text.lower() and 'slam_toolbox' not in text.lower()
