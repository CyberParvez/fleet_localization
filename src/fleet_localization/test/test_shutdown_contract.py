"""Regression checks for clean launch/SIGINT teardown."""
from pathlib import Path


PACKAGE = Path(__file__).parents[1] / 'fleet_localization'


def test_long_running_nodes_handle_external_shutdown_idempotently():
    for filename in ('health.py', 'readiness.py'):
        source = (PACKAGE / filename).read_text(encoding='utf-8')
        assert 'ExternalShutdownException' in source
        assert 'if rclpy.ok()' in source
        assert 'node.destroy_node()' in source
