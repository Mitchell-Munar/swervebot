"""Check estimator input selection and uncertainty conditioning."""
import importlib.util
from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]


def test_planar_input_selection():
    """Keep wheel lateral velocity enabled and exclude standalone laser odometry."""
    params = yaml.safe_load((ROOT / 'config/ekf.yaml').read_text())['ekf_filter_node'][
        'ros__parameters']
    assert all(len(params[key + '_config']) == 15 for key in ('odom0', 'imu0'))
    assert [i for i, v in enumerate(params['odom0_config']) if v] == [6, 7, 11]
    assert not any(key.startswith('odom1') for key in params)
    assert [i for i, v in enumerate(params['imu0_config']) if v] == [11]
    assert params['two_d_mode'] and params['publish_tf']
    assert params['world_frame'] == 'odom'
    assert params['base_link_frame'] == 'base_footprint'


def test_covariance_floors_preserve_uncertainty():
    """Replace unknown zero uncertainty without reducing larger reported errors."""
    spec = importlib.util.spec_from_file_location(
        'conditioning', ROOT / 'scripts/condition_measurements.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    covariance = [0.0] * 36
    covariance[7] = 2.0
    module.covariance_floor(covariance, {0: 0.04, 1: 0.04, 5: 0.01})
    assert covariance[0] == 0.04
    assert covariance[7] == 2.0
    assert covariance[35] == 0.01
