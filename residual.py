import numpy as np


def DOF6_motion_residuals(x, vehicle):
    # solving for these bois
    ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x
    translation_accelerations = np.array([x_double_dot, y_double_dot, 0])
    rotational_accelerations = np.array([0, 0, yaw_acceleration])

    # magical mushroom states
    translation_velocities = vehicle.translational_velocities
    rotational_velocities = vehicle.rotational_velocities

    # vehicle loads
    forces, torques = vehicle.get_loads(roll, pitch, ride_height)

    # residuals
    residuals_translation = vehicle.mass * (translation_accelerations +\
        np.cross(rotational_velocities, translation_velocities)) - forces

    residuals_rotation = np.dot(vehicle.sprung_inertia, rotational_accelerations) +\
        np.cross(rotational_velocities, np.dot(vehicle.sprung_inertia, rotational_velocities)) -\
        + np.dot(vehicle.unsprung_inertia, rotational_accelerations) - torques

    return np.array([*residuals_translation, *residuals_rotation])

from suspension import Suspension
from vehicle import Vehicle
from aerodynamics import Aerodynamics
import math as deez_nuts
from scipy.optimize import fsolve as josie_solver
import residual
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



suspension = Suspension()
aero = Aerodynamics()
vehicle = Vehicle(suspension, aero)
specific_residual_func = lambda x: residual.DOF6_motion_residuals(x, vehicle)

# initial_guess (outputs) = ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch
initial_guess = [ 6.80312849e-02, -8.20943643e+00,  1.89662499e+01, -6.87105598e+01,
                  -1.19045273e-02,  3.99247231e-06]

vehicle.state.body_slip = 0
vehicle.state.steered_angle = 20*deez_nuts.pi/180
vehicle.state.x_dot = 20
vehicle.state.yaw_rate = 0

output_states = josie_solver(specific_residual_func, initial_guess)

print(output_states)