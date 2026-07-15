from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from fleet_localization import map_save
from fleet_localization.map_catalog import MapCatalogError


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
    config=manifest(tmp_path); prepare(monkeypatch,config); count=0
    def call(*_a,**_k):
        nonlocal count; count+=1
        if count == 1: return SimpleNamespace(current_state=SimpleNamespace(label='active'))
        return SimpleNamespace(result=255)
    monkeypatch.setattr(map_save,'_call',call)
    with pytest.raises(RuntimeError,match='result code 255'): map_save.run(args(config))
    assert not (tmp_path/'maps'/'new_map').exists()
    assert not list((tmp_path/'maps').glob('.fleet-map-staging-*'))
    launch=(Path(__file__).parents[1]/'launch'/'localization.launch.py').read_text()
    assert 'map_save' not in launch
