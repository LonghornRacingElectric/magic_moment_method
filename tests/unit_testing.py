import numpy as np
import pandas as pd
import pytest
import warnings
import argparse
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

from pathlib import Path
import sys
path = str(Path(Path(__file__).parent.absolute()).parent.absolute())
sys.path.insert(0, path)
import engine
import vehicle_params

s_dot_sweep = [15]
steering_sweep = [-0.18, 0, 0.18]
body_sweep = [-0.18, 0, 0.18]
reference_file = "tests/test_MMM.csv"
solver = engine.Solver(vehicle_params.UnitTestCar())

@pytest.mark.parametrize("s_dot", s_dot_sweep)
@pytest.mark.parametrize("steered_angle", steering_sweep)
@pytest.mark.parametrize("body_slip", body_sweep)
def test_josie_solver(s_dot, steered_angle, body_slip):
    o_d = solver.solve(engine.State(body_slip, steered_angle, s_dot))
    e_d = pd.read_csv(reference_file)
    e_d_filtered = e_d[((e_d["s_dot"] == s_dot)  & (e_d["steered_angle"] == steered_angle))]
    e_d_filtered = e_d_filtered[e_d_filtered["body_slip"] == body_slip].iloc[0]
    for key, value in e_d_filtered.iteritems():
        if "Unnamed" in str(key):
            continue
        elif type(value) is np.bool_:
            if value != o_d[key]:
                assert f"Failed Getting value {value} but expecting {o_d[key]}"
            continue
        elif abs(o_d[key] - value) > 0.01:
            assert f"Failed Getting value {value} but expecting {o_d[key]}"
    assert True

def generate_test_MMM():
    """
        Generate new testing CSV by running following command:
        python tests/unit_testing.py -g
    """
    log_df = pd.DataFrame()
    for s_dot in s_dot_sweep:
        for body_slip in np.array(steering_sweep):
            for steered_angle in np.array(body_sweep):
                output_dict = solver.solve(engine.State(body_slip, steered_angle, s_dot))
                log_df = pd.concat([log_df, pd.DataFrame([output_dict])], ignore_index=True)
    log_df.to_csv(reference_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--generate", help="Generate test MMM", action = "store_true")
    args = parser.parse_args()
    if args.generate:
        generate_test_MMM()

    #test_josie_solver(15, 0.18, 0.18) # NOTE: use this to investigate why there may be output misalignment