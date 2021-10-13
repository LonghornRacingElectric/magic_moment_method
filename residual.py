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

