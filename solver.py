import numpy as np
from vehicle import Vehicle
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
import math

def main():
    vehicle = Vehicle()
    specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle)

    # initial_guess (outputs) = ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch
    initial_guess = [0.0762, 0, 0, 0, 0, 0]

    state_names = ["x_dot", "body_slip", "steered_angle", "yaw_rate"]
    output_var_names = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
    
    df = None

    peak_slip_angle = 16 * math.pi / 180 # rad

    for x_dot in np.linspace(5, 30, num=6): #np.linspace(7.22,7.22,1):
        for body_slip in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
            for steered_angle in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
                if abs(body_slip + steered_angle) < peak_slip_angle:
                    for yaw_rate in np.linspace(0, 0, num=1):#np.linspace(0.1, 0.1, 1):
                        vehicle.state.body_slip = body_slip
                        vehicle.state.steered_angle = steered_angle
                        vehicle.state.x_dot = x_dot
                        vehicle.state.yaw_rate = yaw_rate

                        output_vars = josie_solver(specific_residual_func, initial_guess)

                        # save data
                        data_dict = copy(vehicle.outputs())
                        data_dict.update(dict(zip(state_names, [x_dot, body_slip, steered_angle, yaw_rate])))
                        data_dict.update(dict(zip(output_var_names, output_vars)))

                        df = pd.DataFrame([data_dict]) if df is None else df.append(data_dict, ignore_index=True)
    
    df.to_csv("MMM.csv")

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
    # TODO: where does this belong?
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