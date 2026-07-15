from pathlib import Path
import yaml


def test_single_reusable_profile_and_required_models():
    root=Path(__file__).parents[1]
    profile=yaml.safe_load((root/'config'/'burger_sim.yaml').read_text())
    amcl=profile['amcl']['ros__parameters']
    assert amcl['robot_model_type']=='nav2_amcl::DifferentialMotionModel'
    assert amcl['laser_model_type']=='likelihood_field'
    assert amcl['tf_broadcast'] is True
    assert not list((root/'config').glob('*robot1*'))
    launch=(root/'launch'/'localization.launch.py').read_text()
    for value in ("interface.frame('map')","interface.frame('odom')","interface.frame('base_footprint')","'scan_topic':'scan'","'use_sim_time':True"):
        assert value in launch
