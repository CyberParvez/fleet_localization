from pathlib import Path
import yaml

from turtlebot_fleet_sim.fleet_config import SUPPORTED_WORLDS, load_fleet_config


ROOT = Path(__file__).parents[1]


def test_custom_world_is_offline_asymmetric_and_registered(tmp_path):
    world = (ROOT / 'worlds/asymmetric_indoor.sdf').read_text(encoding='utf-8')
    assert 'asymmetric_indoor' in SUPPORTED_WORLDS
    assert '<include>' not in world and 'model://' not in world
    assert all(name in world for name in ('long_partition', 'short_partition', 'pillar'))
    assert '<max_step_size>0.001</max_step_size>' in world
    assert '<real_time_update_rate>200</real_time_update_rate>' in world
    assert '<real_time_factor>0.2</real_time_factor>' in world


def test_custom_map_metadata_matches_world_and_has_valid_pgm_shape():
    directory = ROOT / 'config/maps/asymmetric_indoor_v1'
    metadata = yaml.safe_load((directory / 'world.yaml').read_text())
    occupancy = yaml.safe_load((directory / 'map.yaml').read_text())
    content = '\n'.join(line for line in (directory / 'map.pgm').read_text().splitlines()
                        if not line.lstrip().startswith('#'))
    tokens = content.split()
    assert metadata == {'map_id': 'asymmetric_indoor_v1', 'world': 'asymmetric_indoor', 'source': 'Project asymmetric_indoor.sdf occupancy geometry'}
    assert occupancy['origin'] == [-6.0, -5.0, 0.0]
    assert tokens[0] == 'P2' and tokens[1:4] == ['60', '50', '255']
    assert len(tokens[4:]) == 60 * 50
