import numpy as np
from suspension import Suspension
from vehicle import Vehicle
from aerodynamics import Aerodynamics
import math as deez_nuts
from scipy.optimize import fsolve as josie_solver
import pandas as pd
import matplotlib.pyplot as plt

def main():
    vehicle = Vehicle()
    specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle)

    # initial_guess (outputs) = ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch
    initial_guess = [0.07658378838509723, 0, 0, 0, 0, 0]
    data = []
    for x_dot in np.linspace(5,5,1):
        for body_slip in np.linspace(-0.18, 0.18, 21):
            for steered_angle in np.linspace(-0.2,0.2,21):
                for yaw_rate in [0]: #1.7np.linspace(0.1, 0.1, 1):
                    vehicle.state.body_slip = body_slip
                    vehicle.state.steered_angle = steered_angle
                    vehicle.state.x_dot = x_dot
                    vehicle.state.yaw_rate = yaw_rate

                    output_states = josie_solver(specific_residual_func, initial_guess)
                    data.append([x_dot, body_slip, steered_angle, yaw_rate, *output_states])
    #print(data)
    columns = ["x_dot", "body_slip", "steered_angle", "yaw_rate",
                            "ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]

    df = pd.DataFrame(data, columns = columns)
    df.to_csv("export/MMM.csv")

def DOF6_motion_residuals(x, vehicle):
    # solving for these bois
    ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x
    translation_accelerations = np.array([x_double_dot, y_double_dot, 0])
    rotational_accelerations = np.array([0, 0, yaw_acceleration])

    # magical mushroom states
    translation_velocities = vehicle.translational_velocities
    rotational_velocities = vehicle.rotational_velocities

    # vehicle loads
    forces, moments = vehicle.get_loads(roll, pitch, ride_height)

    # kinetic moment
    cg_relative_pos = np.array([0, 0, vehicle.params.cg_total_position[2]])
    kinetic_moment = np.cross(vehicle.params.mass * translation_accelerations, cg_relative_pos)
    moments = np.add(moments, kinetic_moment)

    # residuals
    residuals_translation = vehicle.params.mass * (translation_accelerations +\
        np.cross(rotational_velocities, translation_velocities)) - forces

    residuals_rotation = np.dot(vehicle.params.sprung_inertia, rotational_accelerations) +\
        np.cross(rotational_velocities, np.dot(vehicle.params.sprung_inertia, rotational_velocities)) -\
        + np.dot(vehicle.params.unsprung_inertia, rotational_accelerations) - moments

    return np.array([*residuals_translation, *residuals_rotation])

if __name__ == "__main__":
    main()