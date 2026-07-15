from pathlib import Path
import pytest
import yaml

from fleet_localization.map_catalog import MapCatalogError, resolve_map


MAPS = Path(__file__).parents[2] / 'turtlebot_fleet_sim/config/maps'


def write_manifest(tmp_path, world, map_id):
    path = tmp_path / 'fleet.yaml'
    path.write_text(yaml.safe_dump({
        'fleet': {'map_store': str(MAPS), 'map_id': map_id},
        'simulation': {'world': world, 'gui': False},
        'robots': [{'name': 'robot1', 'namespace': '/robot1', 'frame_prefix': 'robot1',
                    'model': 'burger', 'spawn': {'x': 0.0, 'y': 0.0}}],
    }), encoding='utf-8')
    return path


@pytest.mark.parametrize(('world', 'map_id'), [
    ('turtlebot3_world', 'turtlebot3_world_v1'),
    ('asymmetric_indoor', 'asymmetric_indoor_v1'),
])
def test_bundled_world_map_pairs_resolve(tmp_path, world, map_id):
    resolved = resolve_map(write_manifest(tmp_path, world, map_id))
    assert (resolved.world, resolved.map_id) == (world, map_id)


def test_known_mismatch_names_map_metadata_and_active_world(tmp_path):
    with pytest.raises(MapCatalogError) as caught:
        resolve_map(write_manifest(tmp_path, 'turtlebot3_world', 'asymmetric_indoor_v1'))
    message = str(caught.value)
    assert 'asymmetric_indoor_v1' in message
    assert "'asymmetric_indoor'" in message
    assert "'turtlebot3_world'" in message
