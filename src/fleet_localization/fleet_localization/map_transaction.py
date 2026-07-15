"""Atomic, immutable publication of occupancy maps saved into private staging."""
from pathlib import Path
import ctypes
import errno
import os
import shutil
import tempfile

import yaml

from .map_catalog import MapCatalogError, validate_map_directory, validate_map_id


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
        _write_world_metadata(self.staging / 'world.yaml', {
            'map_id': self.map_id, 'world': self.config.world,
            'source': 'slam_toolbox asynchronous mapping',
        })
        # Validate the complete private directory before it can become visible
        # under its immutable final ID.  Publication itself performs no writes.
        validated = validate_map_directory(self.config, self.map_id, self.staging)
        validated.assert_unchanged()
        # The exclusive mapping-session lock closes the preflight/rename race.
        if self.final.exists() or self.final.is_symlink():
            raise MapCatalogError(f'map ID already exists: {self.map_id!r}')
        _rename_noreplace(self.staging, self.final)
        self.staging = None
        return type(validated)(
            validated.map_id, self.final, self.final / validated.yaml_path.name,
            self.final / validated.image_path.relative_to(validated.directory),
            self.final / validated.metadata_path.name, validated.world,
            tuple((self.final / path.relative_to(validated.directory), size, mtime)
                  for path, size, mtime in validated.fingerprints),
        )

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


def _write_world_metadata(path: Path, metadata: dict):
    path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
