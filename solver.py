import numpy as np
from vehicle import Vehicle
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
import math
from conversions import Conversions

def main():
    vehicle = Vehicle()
    specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle)

    # initial_guess (outputs) = ride_height, x_double_dot, y_double_dot, yaw_accel, roll, pitch
    initial_guess = [Conversions.inch_to_meter(4), 0, 0, 0, 0, 0]
    output_var_names = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
    df = None
    # TODO: better saturation method

    peak_slip_angle = 18 * math.pi / 180 # rad

    # sweep parameters for MMM
    for x_dot in np.linspace(5, 30, num=30): #np.linspace(7.22,7.22,1):
        # for body_slip in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
        #     for steered_angle in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
        for body_slip in [0]:
            for steered_angle in [0]:
                # set vehicle states for each individual sweep
                vehicle.state.body_slip = body_slip
                vehicle.state.steered_angle = steered_angle
                vehicle.state.x_dot = x_dot
                
                # solve for unique output variable set
                output_vars = josie_solver(specific_residual_func, initial_guess)
                
                # see if point is saturated (i.e. all 4 tires slip angles are saturated)
                # saturation will yield to useless data point since it will wrap back around with less acceleration
                # if not saturated, the point will be saved
                if not vehicle.dynamics.tires_saturated:
                    # save data
                    data_dict = copy(vehicle.output_log())
                    data_dict.update(dict(vehicle.state.items()))
                    data_dict.update(dict(zip(output_var_names, output_vars)))
                    df = pd.DataFrame([data_dict]) if df is None else df.append(data_dict, ignore_index=True)
    
    # export data to CSV
    df.to_csv("MMM.csv")

def DOF6_motion_residuals(x, vehicle):
    # solving for these bois
    ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x
    
    # accelerations
    translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
    translation_accelerations_ntb = vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)
    vehicle.outputs.vehicle.accelerations_NTB = translation_accelerations_ntb # for logging purposes
    rotational_accelerations = np.array([0, 0, yaw_acceleration])

    # vehicle loads
    forces, moments = vehicle.get_loads(roll, pitch, ride_height, translation_accelerations_ntb[1])
    vehicle_forces_ntb = vehicle.intermediate_frame_to_ntb_transform(forces)
    vehicle_moments_ntb = vehicle.intermediate_frame_to_ntb_transform(moments)

    # Kinetic moment summation of moments not being done about CG
    # TODO: Make sure sprung inertia is about the intermediate axis
    cg_relative_ntb = np.array([0, 0, vehicle.params.cg_total_position[2]])
    kinetic_moment = np.cross(vehicle.params.mass * translation_accelerations_ntb, cg_relative_ntb)
    
    # solving for summation of forces = m * accel
    summation_forces = vehicle.params.mass * translation_accelerations_ntb - vehicle_forces_ntb
    
    # solving for summation of moments = I * alpha
    # only rotational acceleration being considered is yaw acceleration; which is why it isnt transformed (no roll/pitch accel)
    summation_moments = np.dot(vehicle.params.sprung_inertia, rotational_accelerations) - kinetic_moment - vehicle_moments_ntb

    return np.array([*summation_forces, *summation_moments])

if __name__ == "__main__":
    main()