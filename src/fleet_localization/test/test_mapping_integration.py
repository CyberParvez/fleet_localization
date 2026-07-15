from pathlib import Path
import yaml

from fleet_localization.map_catalog import resolve_map
from fleet_localization.map_store_lock import MapStoreLock
from fleet_localization.map_transaction import MapTransaction


ROOT=Path(__file__).parents[1]


def test_mapping_save_to_later_localization_contract(tmp_path):
    store=tmp_path/'maps'; store.mkdir(); config=tmp_path/'fleet.yaml'
    config.write_text(yaml.safe_dump({'fleet':{'map_store':'./maps','map_id':'old'},
      'simulation':{'world':'turtlebot3_world','gui':False},'robots':[{'name':'robot1','namespace':'/robot1',
      'frame_prefix':'robot1','model':'burger','spawn':{'x':0.0,'y':0.0,'z':0.01,'yaw':0.0}}]}))
    with MapStoreLock.acquire_exclusive(store):
        with MapTransaction(config,'mapped_v2') as transaction:
            (transaction.staging/'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
            (transaction.staging/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':0.05,
              'origin':[0.0,0.0,0.0],'negate':0,'occupied_thresh':0.65,'free_thresh':0.196}))
            transaction.commit()
    with MapStoreLock.acquire_shared(store):
        assert resolve_map(config,'mapped_v2').world == 'turtlebot3_world'
    launch=(ROOT/'launch'/'localization.launch.py').read_text()
    assert "[ekf,slam,slam_lifecycle,health]" in launch
    assert "remappings=[('/map','map'),('/map_metadata','map_metadata')]" in launch
    assert "[ekf,map_server,amcl,lifecycle,health] if mode == 'localization'" in launch
    assert 'MapStoreLock.acquire_exclusive(manifest.map_store)' in launch
    assert 'MapStoreLock.acquire_shared(manifest.map_store)' in launch
