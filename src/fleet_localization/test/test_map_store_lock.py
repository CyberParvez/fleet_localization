import pytest
import multiprocessing
import time
from fleet_localization.map_store_lock import MapStoreBusy, MapStoreLock


def _hold_shared(store, ready):
    lock=MapStoreLock.acquire_shared(store); ready.put(True)
    while True: time.sleep(0.1)


def test_multiple_shared_holders_and_exclusive_conflict(tmp_path):
    first=MapStoreLock.acquire_shared(tmp_path)
    second=MapStoreLock.acquire_shared(tmp_path)
    try:
        with pytest.raises(MapStoreBusy): MapStoreLock.acquire_exclusive(tmp_path)
    finally:
        second.release(); first.release()
    exclusive=MapStoreLock.acquire_exclusive(tmp_path); exclusive.release()


def test_exclusive_holder_rejects_readers_and_writers_on_same_inode(tmp_path):
    exclusive=MapStoreLock.acquire_exclusive(tmp_path)
    try:
        with pytest.raises(MapStoreBusy): MapStoreLock.acquire_shared(tmp_path)
        with pytest.raises(MapStoreBusy): MapStoreLock.acquire_exclusive(tmp_path)
        assert exclusive.path == tmp_path/'.fleet_localization.lock'
    finally: exclusive.release()


def test_context_releases_after_normal_and_exceptional_exit(tmp_path):
    with MapStoreLock(tmp_path,shared=True): pass
    with pytest.raises(RuntimeError):
        with MapStoreLock(tmp_path,shared=True): raise RuntimeError('boom')
    lock=MapStoreLock.acquire_exclusive(tmp_path); lock.release()


def test_operating_system_releases_lock_when_process_exits(tmp_path):
    context=multiprocessing.get_context('spawn'); ready=context.Queue()
    process=context.Process(target=_hold_shared,args=(tmp_path,ready)); process.start()
    assert ready.get(timeout=5) is True
    with pytest.raises(MapStoreBusy): MapStoreLock.acquire_exclusive(tmp_path)
    process.terminate(); process.join(timeout=5)
    assert not process.is_alive()
    lock=MapStoreLock.acquire_exclusive(tmp_path); lock.release()
