"""Controlled workflow regression for the one-robot AMCL launch boundary."""
import importlib.util
from pathlib import Path


ROOT=Path(__file__).parents[1]


def load_launch_module():
    spec=importlib.util.spec_from_file_location('slice03_localization_launch',ROOT/'launch'/'localization.launch.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


class Fingerprint:
    def __init__(self): self.checks=0
    def assert_unchanged(self): self.checks+=1


def test_readiness_boundary_rechecks_map_before_activation():
    launch=load_launch_module(); map_files=Fingerprint(); nodes=[object(),object()]
    assert launch.activation_after_readiness(2,map_files,nodes) is None
    assert map_files.checks == 0
    assert launch.activation_after_readiness(0,map_files,nodes) is nodes
    assert map_files.checks == 1


def test_non_gui_selected_robot_graph_is_readiness_gated_and_robot_scoped():
    text=(ROOT/'launch'/'localization.launch.py').read_text()
    gate=text[text.index('def readiness_exited'):text.index('def localization_exited')]
    assert 'activation_after_readiness(event.returncode,selected_map,nodes)' in gate
    assert 'if activated is not None: return activated' in gate
    assert "nodes=[ekf,map_server,amcl,lifecycle,health]" in gate
    assert "if rviz is not None: nodes.append(rviz)" in gate
    assert "namespace=interface.namespace" in text
    assert "'node_names':['map_server','amcl']" in text
    assert "'global_frame_id':interface.frame('map')" in text
    assert "'odom_frame_id':interface.frame('odom')" in text
    assert "'base_frame_id':interface.frame('base_footprint')" in text
    assert "'scan_topic':'scan'" in text
    assert "DeclareLaunchArgument('rviz',default_value='true')" in text


def test_manual_initialization_and_robot_map_amcl_interfaces_are_exact():
    interface=(ROOT/'fleet_localization'/'interfaces.py').read_text()
    health=(ROOT/'fleet_localization'/'health.py').read_text()
    rviz=(ROOT/'rviz'/'robot_localization.rviz').read_text()
    assert "return f'{self.namespace}/initialpose'" in interface
    assert "return f'{self.namespace}/amcl_pose'" in interface
    assert "return f'{self.namespace}/map'" in interface
    assert 'self.create_subscription(PoseWithCovarianceStamped,self.interface.initialpose_topic' in health
    assert 'self.create_subscription(PoseWithCovarianceStamped,self.interface.amcl_pose_topic' in health
    assert 'Topic: @NS@/initialpose' in rviz
    assert 'Fixed Frame: @PREFIX@/map' in rviz
    assert '/initialpose' not in rviz.replace('@NS@/initialpose','')
