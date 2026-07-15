"""Strict, side-effect-free fleet manifest loading."""

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any

import yaml


SUPPORTED_MODELS = frozenset({'burger'})
SUPPORTED_WORLDS = frozenset({'turtlebot3_world', 'asymmetric_indoor'})
_SAFE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')
_SAFE_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
_TOP_KEYS = frozenset({'fleet', 'simulation', 'robots'})
_FLEET_KEYS = frozenset({'map_store', 'map_id'})
_SIM_KEYS = frozenset({'world', 'gui'})
_ROBOT_KEYS = frozenset({'name', 'namespace', 'frame_prefix', 'model', 'spawn'})
_SPAWN_KEYS = frozenset({'x', 'y', 'z', 'yaw'})


class FleetConfigError(ValueError):
    pass


@dataclass(frozen=True)
class Spawn:
    x: float
    y: float
    z: float = 0.01
    yaw: float = 0.0


@dataclass(frozen=True)
class Robot:
    name: str
    namespace: str
    frame_prefix: str
    model: str
    spawn: Spawn


@dataclass(frozen=True)
class FleetConfig:
    source: Path
    map_store: Path
    map_id: str
    world: str
    gui: bool
    robots: tuple[Robot, ...]


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise FleetConfigError(f'{field} must be a mapping')
    return value


def _keys(value: dict[str, Any], allowed: frozenset[str], field: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise FleetConfigError(f'{field} contains unknown key(s): {", ".join(unknown)}')


def _required(value: dict[str, Any], key: str, field: str) -> Any:
    if key not in value:
        raise FleetConfigError(f'{field}.{key} is required')
    return value[key]


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FleetConfigError(f'{field} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise FleetConfigError(f'{field} must be finite')
    return result


def load_fleet_config(path: str | Path, *, world: str | None = None,
                      gui: bool | None = None) -> FleetConfig:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FleetConfigError(f'fleet_config does not exist: {source}')
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except yaml.YAMLError as exc:
        raise FleetConfigError(f'invalid YAML in {source}: {exc}') from exc
    root = _mapping(document, 'root')
    _keys(root, _TOP_KEYS, 'root')
    fleet = _mapping(_required(root, 'fleet', 'root'), 'fleet')
    simulation = _mapping(_required(root, 'simulation', 'root'), 'simulation')
    _keys(fleet, _FLEET_KEYS, 'fleet')
    _keys(simulation, _SIM_KEYS, 'simulation')

    map_store_raw = _required(fleet, 'map_store', 'fleet')
    map_id = _required(fleet, 'map_id', 'fleet')
    if not isinstance(map_store_raw, str) or not map_store_raw.strip():
        raise FleetConfigError('fleet.map_store must be a nonempty path string')
    if not isinstance(map_id, str) or not _SAFE_ID.fullmatch(map_id):
        raise FleetConfigError('fleet.map_id must be a safe identifier')
    map_store = Path(map_store_raw).expanduser()
    if not map_store.is_absolute():
        map_store = source.parent / map_store
    map_store = map_store.resolve()

    selected_world = world if world not in (None, '') else _required(simulation, 'world', 'simulation')
    if selected_world not in SUPPORTED_WORLDS:
        raise FleetConfigError(f'unsupported simulation.world: {selected_world!r}')
    selected_gui = gui if gui is not None else simulation.get('gui', True)
    if not isinstance(selected_gui, bool):
        raise FleetConfigError('simulation.gui must be boolean')

    raw_robots = _required(root, 'robots', 'root')
    if not isinstance(raw_robots, list) or not raw_robots:
        raise FleetConfigError('robots must be a nonempty list')
    robots: list[Robot] = []
    names: set[str] = set()
    namespaces: set[str] = set()
    prefixes: set[str] = set()
    for index, raw in enumerate(raw_robots):
        field = f'robots[{index}]'
        item = _mapping(raw, field)
        _keys(item, _ROBOT_KEYS, field)
        name = _required(item, 'name', field)
        namespace = _required(item, 'namespace', field)
        prefix = _required(item, 'frame_prefix', field)
        model = _required(item, 'model', field)
        if not isinstance(name, str) or not _SAFE_NAME.fullmatch(name):
            raise FleetConfigError(f'{field}.name must be a valid ROS identifier')
        if not isinstance(namespace, str) or not namespace.startswith('/') or namespace == '/' or namespace.endswith('/'):
            raise FleetConfigError(f'{field}.namespace must be absolute and have no trailing slash')
        if not isinstance(prefix, str) or prefix.startswith('/') or '/' in prefix or not _SAFE_NAME.fullmatch(prefix):
            raise FleetConfigError(f'{field}.frame_prefix must be slash-free with no leading slash')
        if model not in SUPPORTED_MODELS:
            raise FleetConfigError(f'{field}.model is unsupported: {model!r}')
        for value, seen, label in ((name, names, 'name'), (namespace, namespaces, 'namespace'), (prefix, prefixes, 'frame_prefix')):
            if value in seen:
                raise FleetConfigError(f'{field}.{label} duplicates {value!r}')
            seen.add(value)
        spawn_raw = _mapping(_required(item, 'spawn', field), f'{field}.spawn')
        _keys(spawn_raw, _SPAWN_KEYS, f'{field}.spawn')
        spawn = Spawn(
            x=_finite(_required(spawn_raw, 'x', f'{field}.spawn'), f'{field}.spawn.x'),
            y=_finite(_required(spawn_raw, 'y', f'{field}.spawn'), f'{field}.spawn.y'),
            z=_finite(spawn_raw.get('z', 0.01), f'{field}.spawn.z'),
            yaw=_finite(spawn_raw.get('yaw', 0.0), f'{field}.spawn.yaw'),
        )
        for other in robots:
            if math.hypot(spawn.x - other.spawn.x, spawn.y - other.spawn.y) < 0.30:
                raise FleetConfigError(f'{field}.spawn collides with robot {other.name!r} (minimum separation 0.30 m)')
        robots.append(Robot(name, namespace, prefix, model, spawn))
    return FleetConfig(source, map_store, map_id, selected_world, selected_gui, tuple(robots))
