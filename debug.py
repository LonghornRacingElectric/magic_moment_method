from scipy.optimize import broyden1 as josie_solver
import numpy as np
from suspension import Suspension
from vehicle import Vehicle
from aerodynamics import Aerodynamics

def DOF6_motion_residuals(x, vehicle):
    # solving for these bois
    x_double_dot, y_double_dot, yaw_acceleration, roll, pitch, ride_height = x
    translation_accelerations = np.array([x_double_dot, y_double_dot, 0])
    rotational_accelerations = np.array([0, 0, 0])

    # magical mushroom states
    translation_velocities = vehicle.translational_velocities
    rotational_velocities = vehicle.rotational_velocities

    # vehicle loads
    forces, torques = vehicle.get_loads(roll, pitch, ride_height)

    # residuals
    residuals_translation = forces#vehicle.mass * (translation_accelerations +\
        #np.cross(rotational_velocities, translation_velocities)) - forces

    residuals_rotation = np.dot(vehicle.sprung_inertia, rotational_accelerations) +\
        np.cross(rotational_velocities, np.dot(vehicle.sprung_inertia, rotational_velocities)) -\
        + np.dot(vehicle.unsprung_inertia, rotational_accelerations) - torques

    # TODO: REMOVEEE
    # residuals_translation[2] = 0
    residuals_rotation = [0, 0, 0]

    return np.array([*residuals_translation, *residuals_rotation])

# x = x_double_dot, y_double_dot, yaw_acceleration, roll, pitch, ride_height
x = [0, 0, 0, 0, 0, 7.5]

suspension = Suspension()
aero = Aerodynamics()
vehicle = Vehicle(suspension, aero)

# sweep parameters
vehicle.state.body_slip = 0
vehicle.state.steered_angle = 0
vehicle.state.x_dot = 10
vehicle.state.yaw_rate = 0


specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle)
func = lambda x: vehicle.get_loads(*x)
print(josie_solver(func, [0, 0, 7.5], verbose = True))