"""Supported explicit map save command for a live mapping session."""
import argparse
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap

from .interfaces import resolve_robot
from .map_transaction import MapTransaction


def parser():
    result = argparse.ArgumentParser()
    result.add_argument('--fleet-config', required=True)
    result.add_argument('--robot', required=True)
    result.add_argument('--map-id', required=True)
    return result


def _call(node, client, request, label, timeout=10.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f'{label} is unavailable; start the matching mapping session first')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'{label} did not complete successfully')
    return future.result()


def _save_with_replay(node, client, request, topic, timeout=30.0, period=0.1):
    """Replay one retained map so upstream's late volatile saver receives it."""
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('SLAM save service is unavailable; start the matching mapping session first')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    captured = []
    subscription = node.create_subscription(OccupancyGrid, topic,
        lambda message: captured.append(message) if not captured else None, qos)
    deadline = time.monotonic() + 10.0
    while not captured and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    if not captured:
        raise RuntimeError(f'current map is unavailable on {topic}; wait for mapping output')
    publisher = node.create_publisher(OccupancyGrid, topic, qos)
    try:
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            publisher.publish(captured[0])
            rclpy.spin_once(node, timeout_sec=period)
        if not future.done() or future.result() is None:
            raise RuntimeError('SLAM save service did not complete successfully')
        return future.result()
    finally:
        node.destroy_publisher(publisher)


def run(args):
    interface = resolve_robot(args.fleet_config, args.robot)
    node = rclpy.create_node('map_save', namespace=interface.namespace)
    try:
        parameters = AsyncParameterClient(node, 'localization_health')
        if not parameters.wait_for_services(timeout_sec=10.0):
            raise RuntimeError('mapping health service is unavailable; start the matching mapping session first')
        future = parameters.get_parameters(['mode','fleet_config','robot'])
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        values = future.result().values if future.done() and future.result() else []
        actual = [value.string_value for value in values]
        expected = ['mapping', args.fleet_config, args.robot]
        if actual != expected:
            raise RuntimeError(f'active session does not match requested fleet/robot: {actual!r}')
        state = _call(node, node.create_client(GetState, 'slam_toolbox/get_state'), GetState.Request(), 'SLAM lifecycle service')
        if state.current_state.label.lower() != 'active':
            raise RuntimeError('the matching SLAM mapping session is not active')
        with MapTransaction(args.fleet_config, args.map_id) as transaction:
            request = SaveMap.Request()
            request.name.data = str(transaction.save_prefix)
            response = _save_with_replay(node, node.create_client(SaveMap, 'slam_toolbox/save_map'),
                request, interface.map_topic, 30.0)
            if response.result != SaveMap.Response.RESULT_SUCCESS:
                raise RuntimeError(f'SLAM save failed with result code {response.result}')
            resolved = transaction.commit()
            print(f'committed immutable map {resolved.map_id}: {resolved.directory}')
    finally:
        node.destroy_node()


def main(argv=None):
    args = parser().parse_args(argv)
    rclpy.init()
    try:
        run(args)
    except (Exception, ExternalShutdownException) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        if rclpy.ok(): rclpy.shutdown()
