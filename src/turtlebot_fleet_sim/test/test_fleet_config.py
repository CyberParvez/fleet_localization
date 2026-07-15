from copy import deepcopy
import math

import pytest
import yaml

from turtlebot_fleet_sim.fleet_config import FleetConfigError, load_fleet_config


@pytest.fixture
def valid():
    return {
        'fleet': {'map_store': './maps', 'map_id': 'world_v1'},
        'simulation': {'world': 'turtlebot3_world', 'gui': True},
        'robots': [
            {'name': 'robot1', 'namespace': '/robot1', 'frame_prefix': 'robot1',
             'model': 'burger', 'spawn': {'x': -1.0, 'y': 0.0}},
            {'name': 'robot2', 'namespace': '/robot2', 'frame_prefix': 'robot2',
             'model': 'burger', 'spawn': {'x': 1.0, 'y': 0.0, 'yaw': 3.14}},
        ],
    }


def write(tmp_path, value):
    path = tmp_path / 'fleet.yaml'
    path.write_text(yaml.safe_dump(value), encoding='utf-8')
    return path


def test_valid_manifest_and_overrides(tmp_path, valid):
    config = load_fleet_config(write(tmp_path, valid), gui=False)
    assert config.map_store == (tmp_path / 'maps').resolve()
    assert config.world == 'turtlebot3_world'
    assert config.gui is False
    assert [robot.name for robot in config.robots] == ['robot1', 'robot2']
    assert config.robots[0].spawn.z == 0.01


@pytest.mark.parametrize(('mutate', 'message'), [
    (lambda d: d.update({'surprise': 1}), 'unknown key'),
    (lambda d: d['fleet'].pop('map_id'), 'fleet.map_id is required'),
    (lambda d: d['fleet'].update(map_id='../unsafe'), 'safe identifier'),
    (lambda d: d['simulation'].update(world='mars'), 'unsupported simulation.world'),
    (lambda d: d['robots'][0].update(model='waffle'), 'model is unsupported'),
    (lambda d: d['robots'][0].update(namespace='robot1'), 'namespace must be absolute'),
    (lambda d: d['robots'][0].update(frame_prefix='/robot1'), 'slash-free'),
    (lambda d: d['robots'][1].update(name='robot1'), 'duplicates'),
    (lambda d: d['robots'][1].update(namespace='/robot1'), 'duplicates'),
    (lambda d: d['robots'][1].update(frame_prefix='robot1'), 'duplicates'),
    (lambda d: d['robots'][0]['spawn'].update(x=math.inf), 'must be finite'),
    (lambda d: d['robots'][1]['spawn'].update(x=-0.9), 'collides'),
])
def test_required_rejections(tmp_path, valid, mutate, message):
    document = deepcopy(valid)
    mutate(document)
    with pytest.raises(FleetConfigError, match=message):
        load_fleet_config(write(tmp_path, document))


def test_n_robot_schema_is_data_driven(tmp_path, valid):
    valid['robots'].append({
        'name': 'robot3', 'namespace': '/robot3', 'frame_prefix': 'robot3',
        'model': 'burger', 'spawn': {'x': 0.0, 'y': 2.0},
    })
    assert len(load_fleet_config(write(tmp_path, valid)).robots) == 3

