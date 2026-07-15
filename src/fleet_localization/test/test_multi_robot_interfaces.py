"""Exact isolation contract for two selected localization sessions."""
from pathlib import Path

from fleet_localization.interfaces import resolve_robot


ROOT = Path(__file__).parents[2]
FLEET = ROOT / 'turtlebot_fleet_sim' / 'config' / 'fleet.example.yaml'


def test_two_selected_interfaces_have_disjoint_topics_and_frames():
    one = resolve_robot(FLEET, 'robot1')
    two = resolve_robot(FLEET, 'robot2')
    topic_properties = ('wheel_topic', 'imu_topic', 'scan_topic', 'filtered_topic',
                        'map_topic', 'initialpose_topic', 'amcl_pose_topic', 'health_topic')
    one_topics = {getattr(one, name) for name in topic_properties}
    two_topics = {getattr(two, name) for name in topic_properties}
    assert one_topics.isdisjoint(two_topics)
    assert all(topic.startswith('/robot1/') for topic in one_topics)
    assert all(topic.startswith('/robot2/') for topic in two_topics)
    assert one.map_topic == '/robot1/map' and two.map_topic == '/robot2/map'
    assert {one.frame(name) for name in ('map', 'odom', 'base_footprint', 'base_scan')}.isdisjoint(
        {two.frame(name) for name in ('map', 'odom', 'base_footprint', 'base_scan')})


def test_launch_uses_shared_tf_transport_but_selected_robot_frame_edges():
    source = (ROOT / 'fleet_localization' / 'launch' / 'localization.launch.py').read_text()
    for fragment in ("namespace=interface.namespace", "'global_frame_id':interface.frame('map')",
                     "'odom_frame_id':interface.frame('odom')",
                     "'base_frame_id':interface.frame('base_footprint')",
                     "'base_link_frame':interface.frame('base_footprint')"):
        assert fragment in source
    assert "('/tf'" not in source and "('/tf_static'" not in source
    assert "namespace='/" not in source
    assert "frame_id':'map'" not in source and "odom_frame_id':'odom'" not in source


def test_one_owner_per_localization_tf_edge_is_encoded_in_launch():
    source = (ROOT / 'fleet_localization' / 'launch' / 'localization.launch.py').read_text()
    assert source.count("package='robot_localization'") == 1
    assert source.count("package='nav2_amcl'") == 1
    assert 'publish_tf: true' in (ROOT / 'fleet_localization' / 'config' / 'burger_sim.yaml').read_text()
    assert "'tf_broadcast':True" in source
