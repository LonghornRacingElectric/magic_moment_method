import numpy as np
import pandas as pd
# import pytest
from pathlib import Path
import sys

path = str(Path(Path(__file__).parent.absolute()).parent.absolute())
sys.path.insert(0, path)
import engine
import vehicle_params
reference_file = "tests/darren_test_MMM.csv"

s_dot_sweep = [15]
steering_sweep = [-0.18, 0, 0.18]
body_sweep = [-0.18, 0, 0.18]
solver = engine.Solver(vehicle_params.UnitTestCar())

@pytest.mark.parametrize("s_dot", s_dot_sweep)
@pytest.mark.parametrize("steered_angle", steering_sweep)
@pytest.mark.parametrize("body_slip", body_sweep)

def test_darren():
    log_df = pd.DataFrame()
    for s_dot in s_dot_sweep:
        for body_slip in np.array(steering_sweep):
            for steered_angle in np.array(body_sweep):
                output_dict = solver.solve(engine.State(body_slip, steered_angle, s_dot))
                log_df = pd.concat([log_df, pd.DataFrame([output_dict])], ignore_index=True)
    log_df.to_csv(reference_file)

test_darren()