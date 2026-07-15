from pathlib import Path
import pytest
import yaml

from fleet_localization.map_catalog import MapCatalogError, resolve_map


def make_fleet(tmp_path, *, map_id='good', world='turtlebot3_world', metadata_world='turtlebot3_world'):
    directory=tmp_path/'maps'/map_id; directory.mkdir(parents=True)
    (directory/'map.pgm').write_text('P2\n2 2\n255\n0 255 255 0\n')
    (directory/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':0.05,'origin':[0.0,0.0,0.0],'negate':0,'occupied_thresh':0.65,'free_thresh':0.196}))
    (directory/'world.yaml').write_text(yaml.safe_dump({'map_id':map_id,'world':metadata_world,'source':'test fixture'}))
    fleet=tmp_path/'fleet.yaml'
    fleet.write_text(yaml.safe_dump({'fleet':{'map_store':'./maps','map_id':map_id},'simulation':{'world':world,'gui':False},'robots':[{'name':'robot1','namespace':'/robot1','frame_prefix':'robot1','model':'burger','spawn':{'x':0.0,'y':0.0}}]}))
    return fleet,directory


def test_resolves_manifest_relative_map_and_override(tmp_path):
    fleet,directory=make_fleet(tmp_path)
    result=resolve_map(fleet)
    assert result.directory == directory.resolve()
    assert result.image_path.name == 'map.pgm'
    fleet2,_=make_fleet(tmp_path/'second',map_id='override')
    # The override is always resolved only beneath this manifest's own map store.
    with pytest.raises(MapCatalogError,match='does not exist'):
        resolve_map(fleet,'override')


@pytest.mark.parametrize('map_id',['../escape','/absolute','bad id',''])
def test_rejects_unsafe_or_missing_override(tmp_path,map_id):
    fleet,_=make_fleet(tmp_path)
    if map_id == '':
        assert resolve_map(fleet).map_id == 'good'
    else:
        with pytest.raises(MapCatalogError): resolve_map(fleet,map_id)


def test_rejects_world_mismatch_and_missing_assets(tmp_path):
    fleet,directory=make_fleet(tmp_path,metadata_world='other_world')
    with pytest.raises(MapCatalogError,match='active world'): resolve_map(fleet)
    (directory/'world.yaml').write_text(yaml.safe_dump({'map_id':'good','world':'turtlebot3_world','source':'fixture'}))
    (directory/'map.pgm').unlink()
    with pytest.raises(MapCatalogError,match='image does not exist'): resolve_map(fleet)


def test_rejects_image_escape_and_malformed_occupancy(tmp_path):
    fleet,directory=make_fleet(tmp_path)
    (tmp_path/'outside.pgm').write_text('P2\n1 1\n255\n0\n')
    data=yaml.safe_load((directory/'map.yaml').read_text()); data['image']='../../outside.pgm'
    (directory/'map.yaml').write_text(yaml.safe_dump(data))
    with pytest.raises(MapCatalogError,match='escapes map_store'): resolve_map(fleet)
    (directory/'map.yaml').write_text('[]\n')
    with pytest.raises(MapCatalogError,match='must be a YAML mapping'): resolve_map(fleet)


def test_detects_mutation_after_resolution(tmp_path):
    fleet,directory=make_fleet(tmp_path)
    result=resolve_map(fleet)
    (directory/'map.pgm').write_text('P2\n1 1\n255\n0\n')
    with pytest.raises(MapCatalogError,match='changed during'): result.assert_unchanged()
