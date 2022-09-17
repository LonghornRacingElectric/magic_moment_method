import numpy as np
import pandas as pd
from pathlib import Path
import sys

import engine
import vehicle_params

reference_file = "darren_test_MMM.csv"
reference_file2 = "updated_MMM.csv"

s_dot_sweep = [15]
steering_sweep = [-0.18, 0, 0.18]
body_sweep = [-0.18, 0, 0.18]
solver = engine.Solver(vehicle_params.UnitTestCar())


def darren():
    log_df = pd.DataFrame()
    for s_dot in s_dot_sweep:
        for body_slip in np.array(steering_sweep):
            for steered_angle in np.array(body_sweep):
                print("asdf")
                output_dict = solver.solve(engine.State(body_slip, steered_angle, s_dot))
                log_df = pd.concat([log_df, pd.DataFrame([output_dict])], ignore_index=True)
    log_df.to_csv(reference_file)
    updated_df = (log_df[['front_left_tire_tire_centric_forces_1', 'front_right_tire_tire_centric_forces_1',
                          'rear_left_tire_tire_centric_forces_1'
        , 'rear_right_tire_tire_centric_forces_1', 'body_slip', 'steered_angle', 's_dot', 'vehicle_yaw_rate',
                          'rear_left_tire_slip_angle','rear_left_tire_velocity_0','rear_left_tire_velocity_1'
                          ]])
    updated_df.to_csv(reference_file2)
    # with pd.option_context('display.max_rows', None, 'display.max_columns', None):
    #     print(updated_df)


darren()
