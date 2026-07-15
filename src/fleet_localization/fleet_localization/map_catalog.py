"""Strict resolution of immutable occupancy maps from the manifest map store."""
from dataclasses import dataclass
from pathlib import Path
import re

import yaml

from turtlebot_fleet_sim.fleet_config import FleetConfigError, load_fleet_config


_SAFE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')
_MAP_KEYS = frozenset({'image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh', 'mode'})
_META_KEYS = frozenset({'map_id', 'world', 'source'})


class MapCatalogError(FleetConfigError):
    pass


@dataclass(frozen=True)
class ResolvedMap:
    map_id: str
    directory: Path
    yaml_path: Path
    image_path: Path
    metadata_path: Path
    world: str
    fingerprints: tuple[tuple[Path, int, int], ...]

    def assert_unchanged(self) -> None:
        for path, size, mtime_ns in self.fingerprints:
            try:
                stat = path.stat()
            except OSError as exc:
                raise MapCatalogError(f'map {self.map_id!r} changed during the session: {path}') from exc
            if stat.st_size != size or stat.st_mtime_ns != mtime_ns:
                raise MapCatalogError(f'map {self.map_id!r} changed during the session: {path}')


def _document(path: Path, label: str) -> dict:
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MapCatalogError(f'map {label} is unreadable or malformed: {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise MapCatalogError(f'map {label} must be a YAML mapping: {path}')
    return value


def _inside(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve(strict=True)
    if not resolved.is_relative_to(root):
        raise MapCatalogError(f'map {label} escapes map_store: {path}')
    if path.is_symlink():
        raise MapCatalogError(f'map {label} must not be a symlink: {path}')
    return resolved


def resolve_map(fleet_config: str | Path, map_id: str = '') -> ResolvedMap:
    config = load_fleet_config(fleet_config)
    selected = map_id or config.map_id
    if not isinstance(selected, str) or not _SAFE_ID.fullmatch(selected):
        raise MapCatalogError(f'unsafe map ID: {selected!r}')
    store = config.map_store.resolve()
    if not store.is_dir():
        raise MapCatalogError(f'map_store does not exist: {store}')
    directory = store / selected
    if directory.is_symlink():
        raise MapCatalogError(f'map {selected!r} directory must not be a symlink')
    try:
        directory = _inside(directory, store, f'{selected!r} directory')
    except FileNotFoundError as exc:
        raise MapCatalogError(f'map ID does not exist: {selected!r} in {store}') from exc
    if not directory.is_dir():
        raise MapCatalogError(f'map ID is not a directory: {selected!r}')

    yaml_path = directory / 'map.yaml'
    metadata_path = directory / 'world.yaml'
    for path, label in ((yaml_path, 'map.yaml'), (metadata_path, 'world.yaml')):
        if not path.is_file():
            raise MapCatalogError(f'map {selected!r} is missing {label}')
    yaml_path = _inside(yaml_path, directory, 'map.yaml')
    metadata_path = _inside(metadata_path, directory, 'world.yaml')
    occupancy = _document(yaml_path, 'occupancy YAML')
    unknown = sorted(set(occupancy) - _MAP_KEYS)
    required = {'image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'}
    missing = sorted(required - set(occupancy))
    if unknown or missing:
        raise MapCatalogError(f'map {selected!r} occupancy YAML invalid; missing={missing}, unknown={unknown}')
    image = occupancy['image']
    if not isinstance(image, str) or not image or Path(image).is_absolute():
        raise MapCatalogError(f'map {selected!r} image must be a relative path')
    try:
        resolution = float(occupancy['resolution'])
        origin = occupancy['origin']
        occupied = float(occupancy['occupied_thresh'])
        free = float(occupancy['free_thresh'])
    except (TypeError, ValueError) as exc:
        raise MapCatalogError(f'map {selected!r} occupancy values must be numeric') from exc
    if resolution <= 0.0 or not isinstance(origin, list) or len(origin) != 3 or not all(isinstance(v, (int, float)) for v in origin):
        raise MapCatalogError(f'map {selected!r} has invalid resolution or origin')
    if not 0.0 <= free < occupied <= 1.0 or occupancy['negate'] not in (0, 1):
        raise MapCatalogError(f'map {selected!r} has invalid occupancy thresholds or negate')
    image_path = directory / image
    if not image_path.is_file():
        raise MapCatalogError(f'map {selected!r} image does not exist: {image}')
    image_path = _inside(image_path, directory, 'image')

    metadata = _document(metadata_path, 'world metadata')
    unknown_meta = sorted(set(metadata) - _META_KEYS)
    if unknown_meta or set(metadata) != _META_KEYS:
        raise MapCatalogError(f'map {selected!r} world metadata has invalid keys: {unknown_meta}')
    if metadata['map_id'] != selected:
        raise MapCatalogError(f'map metadata ID {metadata["map_id"]!r} does not match {selected!r}')
    if metadata['world'] != config.world:
        raise MapCatalogError(
            f'map {selected!r} is for world {metadata["world"]!r}, active world is {config.world!r}')
    if not isinstance(metadata['source'], str) or not metadata['source'].strip():
        raise MapCatalogError(f'map {selected!r} metadata source must be nonempty')
    files = (yaml_path, image_path, metadata_path)
    fingerprints = tuple((path, path.stat().st_size, path.stat().st_mtime_ns) for path in files)
    return ResolvedMap(selected, directory, yaml_path, image_path, metadata_path, config.world, fingerprints)
