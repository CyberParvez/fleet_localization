from pathlib import Path
import yaml

PROFILE=Path(__file__).parents[1]/'config'/'burger_sim.yaml'
def test_planar_fusion_contract():
    p=yaml.safe_load(PROFILE.read_text())['ekf_filter_node']['ros__parameters']
    assert p['two_d_mode'] is True and p['publish_tf'] is True and p['publish_acceleration'] is False
    assert p['odom0_config']==[False]*6+[True,False,False,False,False,True]+[False]*3
    assert p['imu0_config']==[False]*5+[True]+[False]*5+[True]+[False]*3
    assert sum(p['odom0_config'])==2 and sum(p['imu0_config'])==2
