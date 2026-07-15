from pathlib import Path


def test_gui_only_controls_client_and_default_remains_manifest_driven():
    source = (Path(__file__).parents[1] / 'launch/fleet_sim.launch.py').read_text()
    assert "if config.gui:" in source
    assert "'gz_args': '-g -v2'" in source
    assert "'gz_args': f'-r -s -v2 {world_path}'" in source
    assert "DeclareLaunchArgument('gui', default_value=''" in source

