import numpy as np
import pandas as pd
import itertools
import engine
import vehicle_params
import multiprocessing
import time
from tqdm import tqdm
from time import perf_counter

solver = engine.Solver(vehicle_params.EasyDriver())

peak_slip_angle = 18 * np.pi / 180 # rad
refinement = 5

# s_dot_sweep = [12] # velocity sweep in path tangential direction (total velocity)
# body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
# steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
# torque_request 1
# is_left_bias = np.array([True, False])

#     def __init__(self, body_slip, steered_angle, s_dot, torque_request = 0.0, is_left_bias = True):

x = solver.solve(engine.State(0 * np.pi / 180, 0 * np.pi/180, 12, 0, True))

