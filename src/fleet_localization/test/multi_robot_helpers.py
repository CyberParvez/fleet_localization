"""Process helpers shared by Slice 04 concurrency tests."""
import time

from fleet_localization.map_store_lock import MapStoreLock


def hold_shared(store, ready):
    lock = MapStoreLock.acquire_shared(store)
    ready.put('acquired')
    try:
        while True:
            time.sleep(0.05)
    finally:
        lock.release()


def hold_shared_until_released(store, ready, release):
    with MapStoreLock.acquire_shared(store):
        ready.put('acquired')
        release.wait(timeout=10.0)
