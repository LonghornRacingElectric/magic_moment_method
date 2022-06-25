import numpy as np
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
import pytest
import warnings
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

from pathlib import Path
import sys
path = str(Path(Path(__file__).parent.absolute()).parent.absolute())
sys.path.insert(0, path)
import engine
import vehicle_params

# This whole file is real fucking sloppy right now lol
@pytest.mark.parametrize("s_dot", [15])
@pytest.mark.parametrize("steered_angle", [-0.18, 0, 0.18])
@pytest.mark.parametrize("body_slip", [-0.18, 0, 0.18])
def test_josie_solver(s_dot, steered_angle, body_slip):
    output_var_labels = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
    initial_guess = [0.0762, 0, 0, 0, 0, 0]
    vehicle = engine.Vehicle(vehicle_params.EasyDriver(), engine.State(body_slip, steered_angle, s_dot))
    specific_residual_func = lambda x: engine.Residuals.DOF6_motion_residuals(x, vehicle, output_var_labels)    
    josie_solver(specific_residual_func, initial_guess)

    o_d = copy(vehicle.logger.return_log())
    e_d = pd.read_csv("unit_tests/test_MMM.csv")
    e_d_filtered = e_d[((e_d["s_dot"] == s_dot)  & (e_d["steered_angle"] == steered_angle))]
    e_d_filtered = e_d_filtered[e_d_filtered["body_slip"] == body_slip].iloc[0]
    for key, value in e_d_filtered.iteritems():
        if "Unnamed" in str(key):
            continue
        elif type(value) is np.bool_:
            if value != o_d[key]:
                assert False
            continue
        elif abs(o_d[key] - value) > 0.01:
            assert False
    assert True

def generate_test_MMM():
    output_var_labels = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
    initial_guess = [0.0762, 0, 0, 0, 0, 0]
    vehicle = engine.Vehicle(vehicle_params.EasyDriver())
    specific_residual_func = lambda x: engine.Residuals.DOF6_motion_residuals(x, vehicle, output_var_labels)
    log_df = pd.DataFrame()
    for s_dot in [15]:
        for body_slip in np.array([-0.18, 0, 0.18]):
            for steered_angle in np.array([-0.18, 0, 0.18]):
                vehicle.state = engine.State(body_slip, steered_angle, s_dot)
                josie_solver(specific_residual_func, initial_guess)
                output_dict = copy(vehicle.logger.return_log())
                log_df = pd.concat([log_df, pd.DataFrame([output_dict])], ignore_index=True)
    log_df.to_csv("unit_tests/test_MMM.csv")

if __name__ == "__main__":
    pass
    #generate_test_MMM()
    #test_josie_solver(15, 0.18, 0.18)