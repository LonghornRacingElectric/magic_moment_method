import numpy as np
import pandas as pd
import pytest
import warnings
import argparse
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

import sys
sys.path.append("..")
import magic_moment_method.vehicle_params as vehicle_params
import magic_moment_method.state_solver as state_solver

s_dot_sweep = [15]
steering_sweep = [-0.07, 0, 0.07]
body_sweep = [-0.07, 0, 0.07]
torque_sweep = [0]
differential_bias_sweep = [True]
reference_file = "tests/test_MMM.csv"
solver = state_solver.Solver(vehicle_params.UnitTestCar())

@pytest.mark.parametrize("s_dot", s_dot_sweep)
@pytest.mark.parametrize("steered_angle", steering_sweep)
@pytest.mark.parametrize("body_slip", body_sweep)
@pytest.mark.parametrize("torque_request", torque_sweep)
@pytest.mark.parametrize("is_left_diff_bias", differential_bias_sweep)
def test_josie_solver(s_dot, steered_angle, body_slip, torque_request, is_left_diff_bias):
    o_d = solver.solve(state_solver.State(body_slip, steered_angle, s_dot, torque_request, is_left_diff_bias))
    e_d = pd.read_csv(reference_file)
    try:
        e_d_filtered = e_d[(e_d["body_slip"] == body_slip) & (e_d["s_dot"] == s_dot) & (e_d["steered_angle"] == steered_angle)].iloc[0]
    except:
        return True

    for key, value in e_d_filtered.items():
        if "Unnamed" in str(key) or value is np.NaN or key == '0':
            continue
        elif type(value) in [np.bool_, bool]:
            if value != o_d[key]:
                pytest.fail(f"Failed Getting value {value} but expecting {o_d[key]}")
            continue
        elif abs(o_d[key] - value) > 0.01:
            pytest.fail(f"Failed Getting value {value} but expecting {o_d[key]} for {key}")
    return True

def generate_test_MMM():
    """
        Generate new testing CSV by running following command:
        python tests/unit_testing.py -g
    """
    log_df = pd.DataFrame()
    for s_dot in s_dot_sweep:
        for body_slip in np.array(body_sweep):
            for steered_angle in np.array(steering_sweep):
                for torque_req in np.array(torque_sweep):
                    for bias in np.array(differential_bias_sweep):
                        output_dict = solver.solve(state_solver.State(body_slip, steered_angle, s_dot, torque_req, bias))
                        log_df = pd.concat([log_df, pd.DataFrame([output_dict])], ignore_index=True)
    log_df.to_csv(reference_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--generate", help="Generate test MMM", action = "store_true")
    args = parser.parse_args()
    if args.generate:
        generate_test_MMM()

    #test_josie_solver(15, .07, 0, 0, True) # NOTE: use this to investigate why there may be output misalignment