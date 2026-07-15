from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from fleet_localization import map_save
from fleet_localization.map_catalog import MapCatalogError
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


def manifest(tmp_path):
    (tmp_path/'maps').mkdir()
    path=tmp_path/'fleet.yaml'
    path.write_text(yaml.safe_dump({'fleet':{'map_store':'./maps','map_id':'old'},
      'simulation':{'world':'turtlebot3_world','gui':False},'robots':[{'name':'robot1','namespace':'/robot1',
      'frame_prefix':'robot1','model':'burger','spawn':{'x':0.0,'y':0.0,'z':0.01,'yaw':0.0}}]}))
    return path


class Future:
    def __init__(self,values): self.values=values
    def done(self): return True
    def result(self): return SimpleNamespace(values=[SimpleNamespace(string_value=v) for v in self.values])


class Parameters:
    def __init__(self,_node,_target): pass
    def wait_for_services(self,timeout_sec): return True
    def get_parameters(self,names): return Future(self.values)


class Node:
    def create_client(self,*_args): return object()
    def destroy_node(self): pass


def prepare(monkeypatch,config,identity=None):
    Parameters.values=identity or ['mapping',str(config),'robot1']
    monkeypatch.setattr(map_save,'AsyncParameterClient',Parameters)
    monkeypatch.setattr(map_save.rclpy,'create_node',lambda *_a,**_k:Node())
    monkeypatch.setattr(map_save.rclpy,'spin_until_future_complete',lambda *_a,**_k:None)
    monkeypatch.setattr(map_save,'_save_with_replay',lambda *_a,**_k:SimpleNamespace(result=0))


def args(config,map_id='new_map'):
    return SimpleNamespace(fleet_config=str(config),robot='robot1',map_id=map_id)


def test_cli_requires_identity_and_exposes_no_raw_path_or_force():
    parser=map_save.parser()
    with pytest.raises(SystemExit): parser.parse_args([])
    parsed=parser.parse_args(['--fleet-config','f','--robot','r','--map-id','m'])
    assert vars(parsed) == {'fleet_config':'f','robot':'r','map_id':'m'}
    assert '--output' not in parser.format_help() and '--force' not in parser.format_help()


def test_session_identity_and_active_lifecycle_gate_precede_save(tmp_path,monkeypatch):
    config=manifest(tmp_path); prepare(monkeypatch,config,['localization',str(config),'robot1'])
    monkeypatch.setattr(map_save,'_call',lambda *_a,**_k:pytest.fail('SLAM must not be called'))
    with pytest.raises(RuntimeError,match='does not match'): map_save.run(args(config))
    prepare(monkeypatch,config)
    monkeypatch.setattr(map_save,'_call',lambda *_a,**_k:SimpleNamespace(current_state=SimpleNamespace(label='inactive')))
    with pytest.raises(RuntimeError,match='not active'): map_save.run(args(config))


def test_existing_and_unsafe_ids_fail_before_upstream_save(tmp_path,monkeypatch):
    config=manifest(tmp_path); (tmp_path/'maps'/'existing').mkdir(); prepare(monkeypatch,config)
    calls=[]
    def call(*_a,**_k):
        calls.append(True); return SimpleNamespace(current_state=SimpleNamespace(label='active'))
    monkeypatch.setattr(map_save,'_call',call)
    with pytest.raises(MapCatalogError,match='already exists'): map_save.run(args(config,'existing'))
    assert len(calls) == 1
    calls.clear()
    with pytest.raises(MapCatalogError,match='unsafe map ID'): map_save.run(args(config,'../escape'))
    assert len(calls) == 1


def test_upstream_failure_cleans_staging_and_no_shutdown_save_exists(tmp_path,monkeypatch):
    config=manifest(tmp_path); prepare(monkeypatch,config)
    monkeypatch.setattr(map_save,'_call',lambda *_a,**_k:SimpleNamespace(current_state=SimpleNamespace(label='active')))
    monkeypatch.setattr(map_save,'_save_with_replay',lambda *_a,**_k:SimpleNamespace(result=255))
    with pytest.raises(RuntimeError,match='result code 255'): map_save.run(args(config))
    assert not (tmp_path/'maps'/'new_map').exists()
    assert not list((tmp_path/'maps').glob('.fleet-map-staging-*'))
    launch=(Path(__file__).parents[1]/'launch'/'localization.launch.py').read_text()
    assert 'map_save' not in launch


def test_replay_uses_exact_topic_qos_and_cleans_transport(monkeypatch):
    events=[]
    class SaveFuture:
        def done(self): return len(events) >= 2 and events.count('publish') >= 2
        def result(self): return SimpleNamespace(result=0)
    class Client:
        def wait_for_service(self,timeout_sec): return True
        def call_async(self,request): return SaveFuture()
    class ReplayNode:
        def create_subscription(self,msg_type,topic,callback,qos):
            events.append(('subscription',msg_type,topic,qos)); self.callback=callback; return 'sub'
        def destroy_subscription(self,value): events.append(('destroy_subscription',value))
        def create_publisher(self,msg_type,topic,qos):
            events.append(('publisher',msg_type,topic,qos))
            return SimpleNamespace(publish=lambda message:events.append('publish'))
        def destroy_publisher(self,value): events.append('destroy_publisher')
    node=ReplayNode(); message=OccupancyGrid()
    def spin(*_args,**_kwargs):
        if hasattr(node,'callback'): node.callback(message); del node.callback
    monkeypatch.setattr(map_save.rclpy,'spin_once',spin)
    response=map_save._save_with_replay(node,Client(),object(),'/robot1/map',period=0.0)
    assert response.result == 0 and events.count('publish') == 2
    subscription=events[0]
    assert subscription[1:3] == (OccupancyGrid,'/robot1/map')
    assert subscription[3].reliability == ReliabilityPolicy.RELIABLE
    assert subscription[3].durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert 'destroy_publisher' in events and ('destroy_subscription','sub') in events


def test_replay_fails_clearly_and_cleans_when_no_map(monkeypatch):
    node=SimpleNamespace(create_subscription=lambda *_a,**_k:'sub',
        destroy_subscription=lambda value:setattr(node,'destroyed',value))
    ticks=iter((0.0,11.0))
    monkeypatch.setattr(map_save.time,'monotonic',lambda:next(ticks))
    monkeypatch.setattr(map_save.rclpy,'spin_once',lambda *_a,**_k:None)
    client=SimpleNamespace(wait_for_service=lambda timeout_sec:True)
    with pytest.raises(RuntimeError,match='current map is unavailable'):
        map_save._save_with_replay(node,client,object(),'/robot1/map')
    assert node.destroyed == 'sub'
