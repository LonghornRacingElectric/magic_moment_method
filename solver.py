import numpy as np
from better_namespace import BetterNamespace
from engine.vehicle import Vehicle
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
import math
from vehicle_params.easy_driver import EasyDriver

def main():
    state = BetterNamespace()
    vehicle = Vehicle(EasyDriver(), state)
    specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle)

    # initial_guess (outputs) = ride_height, x_double_dot, y_double_dot, yaw_accel, roll, pitch
    initial_guess = [0.0762, 0, 0, 0, 0, 0]
    
    df = None

    peak_slip_angle = 18 * math.pi / 180 # rad

    # sweep parameters for MMM
    for s_dot in [15]: #np.linspace(7.22,7.22,1):
        for body_slip in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
            for steered_angle in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
                # set vehicle states for each individual sweep
                state.body_slip = body_slip
                state.steered_angle = steered_angle
                state.s_dot = s_dot # total velocity in body slip direction
                
                # solve for unique output variable set
                output_vars = josie_solver(specific_residual_func, initial_guess)
                output_dict = dict(zip(["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"], output_vars))
                output_dict.update(copy(vehicle.output_log()))
                
                # see if point is saturated (i.e. all 4 tires slip angles are saturated)
                # saturation will yield to useless data point since it will wrap back around with less acceleration
                # if not saturated, the point will be saved
                # TODO: Better saturation method implementation in tires
                if not output_dict["dynamics_tires_saturated"] and not output_dict["dynamics_two_tires_lifting"]:
                    df = pd.DataFrame([output_dict]) if df is None else df.append(output_dict, ignore_index=True)
                    #print(vehicle.aero.outputs.forces)
    
    # export data to CSV
    df.to_csv("analysis/MMM.csv")

def DOF6_motion_residuals(x, vehicle):
    # solving for these bois
    ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x
    
    # accelerations
    translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
    translation_accelerations_ntb = vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)
    vehicle.log_linear_accelerations_ntb(translation_accelerations_ntb)

    # vehicle loads
    forces, moments = vehicle.get_loads(roll, pitch, ride_height, translation_accelerations_ntb[1])
    vehicle_forces_ntb = vehicle.intermediate_frame_to_ntb_transform(forces)
    vehicle_moments_ntb = vehicle.intermediate_frame_to_ntb_transform(moments)

    # Kinetic moment summation of moments not being done about CG
    # TODO: Make sure sprung inertia is about the intermediate axis
    # TODO: CoG movements
    kinetic_moments = vehicle.get_kinetic_moments(translation_accelerations_ntb) 
    
    # solving for summation of forces = m * accel
    summation_forces = vehicle.get_inertial_forces(translation_accelerations_ntb) - vehicle_forces_ntb
    
    # solving for summation of moments = I * alpha
    # only rotational acceleration being considered is yaw acceleration; which is why it isnt transformed (no roll/pitch accel)
    yaw_moment = vehicle.get_yaw_moment(yaw_acceleration)
    summation_moments = np.array([0, 0, yaw_moment]) - kinetic_moments - vehicle_moments_ntb

    return np.array([*summation_forces, *summation_moments])

if __name__ == "__main__":
    main()