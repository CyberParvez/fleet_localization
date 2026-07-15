"""Advisory process locks for immutable map-store sessions."""
from pathlib import Path
import fcntl


class MapStoreBusy(RuntimeError):
    pass


class MapStoreLock:
    def __init__(self, store: str | Path, *, shared: bool):
        self.store = Path(store).resolve()
        self.shared = shared
        self._file = None

    @classmethod
    def acquire_shared(cls, store: str | Path):
        return cls(store, shared=True).acquire()

    @classmethod
    def acquire_exclusive(cls, store: str | Path):
        return cls(store, shared=False).acquire()

    def acquire(self):
        if self._file is not None:
            return self
        self._file = (self.store / '.fleet_localization.lock').open('a+b')
        operation = fcntl.LOCK_SH if self.shared else fcntl.LOCK_EX
        try:
            fcntl.flock(self._file.fileno(), operation | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._file.close()
            self._file = None
            kind = 'shared localization' if self.shared else 'exclusive mapping'
            raise MapStoreBusy(f'cannot acquire {kind} lock for map_store {self.store}') from exc
        return self

    def release(self):
        if self._file is not None:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            self._file.close()
            self._file = None

    def __enter__(self):
        return self.acquire()

    def __exit__(self, *_args):
        self.release()
