"""Generate a namespaced Burger SDF from the installed official asset."""

from pathlib import Path
import xml.etree.ElementTree as ET

from .fleet_config import Robot


def render_burger_sdf(upstream_path: str | Path, robot: Robot) -> str:
    """Apply the narrow identity/topic override; geometry and sensors remain upstream."""
    root = ET.parse(upstream_path).getroot()
    model = root.find('model')
    if model is None:
        raise ValueError('official Burger SDF contains no model')
    model.set('name', robot.name)
    prefix = robot.frame_prefix + '/'
    link_names = {element.get('name') for element in model.findall('link')}
    joint_names = {element.get('name') for element in model.findall('joint')}
    for element in model.findall('link'):
        element.set('name', prefix + element.get('name'))
    for element in model.findall('joint'):
        element.set('name', prefix + element.get('name'))
        for tag in ('parent', 'child'):
            child = element.find(tag)
            if child is not None and child.text in link_names:
                child.text = prefix + child.text
    for plugin in model.findall('plugin'):
        for element in list(plugin):
            if element.tag in ('left_joint', 'right_joint', 'joint_name') and element.text in joint_names:
                element.text = prefix + element.text
    for sensor in model.findall('.//sensor'):
        frame = sensor.find('gz_frame_id')
        if frame is None:
            frame = ET.SubElement(sensor, 'gz_frame_id')
        parent_link = next((link for link in model.findall('link') if sensor in list(link)), None)
        if parent_link is not None:
            frame.text = parent_link.get('name')

    topics = {
        'cmd_vel': f'{robot.namespace}/_sim/cmd_vel',
        'odom': f'{robot.namespace}/_sim/odom',
        'imu': f'{robot.namespace}/_sim/imu',
        'scan': f'{robot.namespace}/_sim/scan',
        'joint_states': f'{robot.namespace}/joint_states',
    }
    for tag in ('topic', 'odom_topic'):
        for element in model.findall(f'.//{tag}'):
            if element.text in topics:
                element.text = topics[element.text]
    diff_drive = next((p for p in model.findall('plugin') if 'DiffDrive' in p.get('name', '')), None)
    if diff_drive is None:
        raise ValueError('official Burger SDF contains no DiffDrive plugin')
    for tag, value in (
        ('frame_id', prefix + 'odom'),
        ('child_frame_id', prefix + 'base_footprint'),
        # Gazebo may publish this internal transform, but it is intentionally never bridged to ROS.
        ('tf_topic', f'{robot.namespace}/_sim/unused_odom_tf'),
    ):
        element = diff_drive.find(tag)
        if element is None:
            element = ET.SubElement(diff_drive, tag)
        element.text = value
    return ET.tostring(root, encoding='unicode')
