"""Atomic, immutable publication of occupancy maps saved into private staging."""
from pathlib import Path
import ctypes
import errno
import os
import shutil
import tempfile

import yaml

from .map_catalog import MapCatalogError, resolve_map, validate_map_id


class MapTransaction:
    def __init__(self, fleet_config, map_id):
        from turtlebot_fleet_sim.fleet_config import load_fleet_config
        self.fleet_config = str(Path(fleet_config).resolve())
        self.config = load_fleet_config(self.fleet_config)
        self.map_id = validate_map_id(map_id)
        self.store = self.config.map_store.resolve()
        self.final = self.store / self.map_id
        self.staging = None

    def __enter__(self):
        if not self.store.is_dir():
            raise MapCatalogError(f'map_store does not exist: {self.store}')
        if self.final.exists() or self.final.is_symlink():
            raise MapCatalogError(f'map ID already exists: {self.map_id!r}')
        self.staging = Path(tempfile.mkdtemp(prefix='.fleet-map-staging-', dir=self.store))
        return self

    @property
    def save_prefix(self):
        return self.staging / 'map'

    def commit(self):
        yaml_path = self.staging / 'map.yaml'
        if not yaml_path.is_file():
            raise MapCatalogError('SLAM save did not produce map.yaml')
        try:
            occupancy = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as exc:
            raise MapCatalogError(f'SLAM map YAML is invalid: {exc}') from exc
        if not isinstance(occupancy, dict) or not isinstance(occupancy.get('image'), str):
            raise MapCatalogError('SLAM map YAML has no relative image')
        image = Path(occupancy['image'])
        if image.is_absolute() or '..' in image.parts:
            raise MapCatalogError('SLAM map image escapes staging directory')
        image_path = self.staging / image
        if image_path.is_symlink() or not image_path.is_file() or not image_path.resolve().is_relative_to(self.staging.resolve()):
            raise MapCatalogError('SLAM map image is missing or unsafe')
        (self.staging / 'world.yaml').write_text(yaml.safe_dump({
            'map_id': self.map_id, 'world': self.config.world,
            'source': 'slam_toolbox asynchronous mapping',
        }, sort_keys=False), encoding='utf-8')
        # The exclusive mapping-session lock closes the preflight/rename race.
        if self.final.exists() or self.final.is_symlink():
            raise MapCatalogError(f'map ID already exists: {self.map_id!r}')
        _rename_noreplace(self.staging, self.final)
        self.staging = None
        try:
            return resolve_map(self.fleet_config, self.map_id)
        except Exception:
            # Never leave a final directory that failed canonical validation.
            shutil.rmtree(self.final, ignore_errors=True)
            raise

    def __exit__(self, *_args):
        if self.staging is not None:
            shutil.rmtree(self.staging, ignore_errors=True)


def _rename_noreplace(source: Path, destination: Path):
    """Linux same-filesystem atomic directory publication without replacement."""
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, 'renameat2', None)
    if renameat2 is None:
        raise MapCatalogError('filesystem runtime does not provide atomic no-replace rename')
    result = renameat2(-100, os.fsencode(source), -100, os.fsencode(destination), 1)
    if result != 0:
        code = ctypes.get_errno()
        if code == errno.EEXIST:
            raise MapCatalogError(f'map ID already exists: {destination.name!r}')
        raise OSError(code, os.strerror(code), destination)
