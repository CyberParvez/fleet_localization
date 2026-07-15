from pathlib import Path
import xml.etree.ElementTree as ET

from turtlebot_fleet_sim.fleet_config import Robot, Spawn
from turtlebot_fleet_sim.model import render_burger_sdf


SDF = '''<sdf version="1.8"><model name="turtlebot3_burger">
<link name="base_footprint"/><link name="base_link"/>
<link name="imu_link"><sensor name="imu" type="imu"><topic>imu</topic></sensor></link>
<link name="base_scan"><sensor name="lidar" type="gpu_lidar"><topic>scan</topic><gz_frame_id>base_scan</gz_frame_id></sensor></link>
<link name="wheel_left_link"/><link name="wheel_right_link"/>
<joint name="base_joint" type="fixed"><parent>base_footprint</parent><child>base_link</child></joint>
<joint name="wheel_left_joint" type="revolute"><parent>base_link</parent><child>wheel_left_link</child></joint>
<joint name="wheel_right_joint" type="revolute"><parent>base_link</parent><child>wheel_right_link</child></joint>
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
<left_joint>wheel_left_joint</left_joint><right_joint>wheel_right_joint</right_joint>
<topic>cmd_vel</topic><odom_topic>odom</odom_topic><frame_id>odom</frame_id>
<child_frame_id>base_footprint</child_frame_id><tf_topic>/tf</tf_topic></plugin>
<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
<topic>joint_states</topic><joint_name>wheel_left_joint</joint_name></plugin>
</model></sdf>'''


def test_model_override_is_namespaced_and_suppresses_ros_odom_tf(tmp_path: Path):
    source = tmp_path / 'model.sdf'
    source.write_text(SDF, encoding='utf-8')
    robot = Robot('robot1', '/robot1', 'robot1', 'burger', Spawn(0.0, 0.0))
    rendered = render_burger_sdf(source, robot)
    for frame in ('base_footprint', 'base_link', 'base_scan', 'imu_link',
                  'wheel_left_link', 'wheel_right_link'):
        assert f'name="robot1/{frame}"' in rendered
    assert '/robot1/_sim/cmd_vel' in rendered
    assert '/robot1/_sim/odom' in rendered
    assert '/robot1/_sim/unused_odom_tf' in rendered
    assert '<tf_topic>/tf</tf_topic>' not in rendered
    assert '<frame_id>robot1/odom</frame_id>' in rendered
    assert '<child_frame_id>robot1/base_footprint</child_frame_id>' in rendered


def test_actual_turtlebot3_237_asset_has_every_reference_prefixed():
    candidates = (
        Path('/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'),
        Path('/tmp/tb3pkgs/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'),
    )
    upstream = next((path for path in candidates if path.exists()), None)
    if upstream is None:
        import pytest
        pytest.skip('official TurtleBot3 2.3.7 asset is unavailable')
    robot = Robot('robot9', '/robot9', 'robot9', 'burger', Spawn(0.0, 0.0))
    model = ET.fromstring(render_burger_sdf(upstream, robot)).find('model')
    assert model is not None
    links = {element.get('name') for element in model.findall('link')}
    joints = {element.get('name') for element in model.findall('joint')}
    assert links and joints
    assert all(name.startswith('robot9/') for name in links | joints)
    for joint in model.findall('joint'):
        assert joint.findtext('parent') in links
        assert joint.findtext('child') in links
    assert model.findtext("plugin[@name='gz::sim::systems::DiffDrive']/tf_topic") == '/robot9/_sim/unused_odom_tf'
    assert model.findtext("plugin[@name='gz::sim::systems::DiffDrive']/topic") == '/robot9/_sim/cmd_vel'
