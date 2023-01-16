import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import seaborn as sns
sys.path.append(".")
import vehicle_params
# import state_solver
from magic_moment_method import solver_sweeper
import labellines

mesh = 11 # NOTE: MAKE SURE THIS IS ODD

# NOTE: guesstimation based from TTC on maximum tire saturation slip angle
sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
        "steered_angle" : (-20 * np.pi / 180, 20 * np.pi / 180),
        "velocity" : (3, 30),
        "torque_request": (-1, 1),
        "is_left_diff_bias": (True)}

body_slip_sweep = np.linspace(sweep_range["body_slip"][0], sweep_range["body_slip"][1], mesh)
steered_angle_sweep = np.linspace(sweep_range["steered_angle"][0], sweep_range["steered_angle"][1], mesh)
torque_sweep = np.linspace(sweep_range["torque_request"][0], sweep_range["torque_request"][1], mesh)

sweep_values = {"velocity": [12], "body_slip":body_slip_sweep, "steered_angle":steered_angle_sweep,
                 "torque_request": [0], "is_left_diff_bias" : [True, False]}


vehicle = vehicle_params.Concept2023(motor_directory="vehicle_params/Eff228.csv")


df = solver_sweeper(vehicle, sweep_values)

tires = ["front_left", "front_right", "rear_left", "rear_right"]