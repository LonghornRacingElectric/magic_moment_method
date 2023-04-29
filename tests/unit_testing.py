import numpy as np
import warnings
warnings.filterwarnings('ignore', 'The iteration is not making good progress')
import sys
sys.path.append("..")
import magic_moment_method.vehicle_params as vehicle_params
from magic_moment_method.solver_sweeper import solver_sweeper

def test_no_fail():
    mesh = 5
    sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-20 * np.pi / 180, 20 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1)}
    body_slip_sweep = np.linspace(sweep_range["body_slip"][0], sweep_range["body_slip"][1], mesh)
    steered_angle_sweep = np.linspace(sweep_range["steered_angle"][0], sweep_range["steered_angle"][1], mesh)
    torque_sweep = np.linspace(sweep_range["torque_request"][0], sweep_range["torque_request"][1], mesh)
    sweep_values = {"velocity": [12], "body_slip":body_slip_sweep, "steered_angle":steered_angle_sweep,
                    "torque_request": torque_sweep, "is_left_diff_bias" : [True, False]}
    vehicle = vehicle_params.Concept2023(motor_directory="vehicle_params/Eff228.csv")
    _ = solver_sweeper(vehicle, sweep_values)