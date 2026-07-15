"""Process-level proof for concurrent localization map readers."""
import multiprocessing

import pytest

from fleet_localization.map_store_lock import MapStoreBusy, MapStoreLock
from multi_robot_helpers import hold_shared, hold_shared_until_released


def _spawn(target, *args):
    context = multiprocessing.get_context('spawn')
    process = context.Process(target=target, args=args)
    process.start()
    return process


def test_two_process_readers_release_independently_and_block_writer(tmp_path):
    context = multiprocessing.get_context('spawn')
    first_ready, second_ready = context.Queue(), context.Queue()
    first_release, second_release = context.Event(), context.Event()
    first = _spawn(hold_shared_until_released, tmp_path, first_ready, first_release)
    second = _spawn(hold_shared_until_released, tmp_path, second_ready, second_release)
    try:
        assert first_ready.get(timeout=5) == 'acquired'
        assert second_ready.get(timeout=5) == 'acquired'
        with pytest.raises(MapStoreBusy):
            MapStoreLock.acquire_exclusive(tmp_path)

        first_release.set(); first.join(timeout=5)
        assert first.exitcode == 0 and second.is_alive()
        with pytest.raises(MapStoreBusy):
            MapStoreLock.acquire_exclusive(tmp_path)

        second_release.set(); second.join(timeout=5)
        assert second.exitcode == 0
        MapStoreLock.acquire_exclusive(tmp_path).release()
    finally:
        for process in (first, second):
            if process.is_alive():
                process.terminate(); process.join(timeout=5)


def test_crashed_reader_does_not_release_other_reader(tmp_path):
    context = multiprocessing.get_context('spawn')
    first_ready, second_ready = context.Queue(), context.Queue()
    first = _spawn(hold_shared, tmp_path, first_ready)
    second = _spawn(hold_shared, tmp_path, second_ready)
    try:
        assert first_ready.get(timeout=5) == second_ready.get(timeout=5) == 'acquired'
        first.terminate(); first.join(timeout=5)
        assert first.exitcode is not None and second.is_alive()
        with pytest.raises(MapStoreBusy):
            MapStoreLock.acquire_exclusive(tmp_path)
        second.terminate(); second.join(timeout=5)
        MapStoreLock.acquire_exclusive(tmp_path).release()
    finally:
        for process in (first, second):
            if process.is_alive():
                process.terminate(); process.join(timeout=5)
