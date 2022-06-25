import numpy as np
from helpers.better_namespace import BetterNamespace
from engine.vehicle import Vehicle
from engine.state import State
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
import math
from vehicle_params.easy_driver import EasyDriver
import warnings
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

def main():
    # These are the output variables being solved for to match the prescribed states!
    output_var_labels = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
    initial_guess = [0.0762, 0, 0, 0, 0, 0]
    vehicle = Vehicle(EasyDriver())
    specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle, output_var_labels)
    log_df = None

    # sweep parameters for MMM; any parameter in the vehicle_params file can be swept as well.
    for s_dot in [15]:
        
        # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
        peak_slip_angle = 18 * math.pi / 180 # rad 
        
        for body_slip in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
            for steered_angle in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
                # set prescribed vehicle states for each parameter sweep
                vehicle.state.body_slip = body_slip
                vehicle.state.steered_angle = steered_angle
                vehicle.state.s_dot = s_dot # total velocity in body slip direction
                
                # solve for unique output variable set to match the prescribed states, with an initial guess of outputs
                josie_solver(specific_residual_func, initial_guess)
                
                 # ~~~ Handles Post Processing of Solution ~~~ #
                output_dict = copy(vehicle.logger.return_log())
                
                # see if point is saturated (i.e. all 4 tires slip angles are saturated)
                # saturation will yield to useless data point since it will wrap back around with less acceleration
                # if not saturated, the point will be saved
                # TODO: Should 1 tire even be allowed to lift???
                if not output_dict["dynamics_tires_saturated"] and not output_dict["dynamics_two_tires_lifting"]:
                    log_df = pd.DataFrame([output_dict]) if log_df is None else log_df.append(output_dict, ignore_index=True)
    
    # add solved outputs to CSV file along with intermediate logged values
    log_df.to_csv("analysis/MMM.csv")
    print("Export successful to CSV, MMM complete!")

def DOF6_motion_residuals(x, vehicle, output_var_labels):
    # solving for these bois
    ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x
    
    # accelerations
    translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
    translation_accelerations_ntb = vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)

    # vehicle loads
    yaw_rate = vehicle.get_yaw_rate(translation_accelerations_ntb[1])
    forces, moments = vehicle.get_loads(roll, pitch, ride_height, yaw_rate)
    vehicle_forces_ntb = vehicle.intermediate_frame_to_ntb_transform(forces)
    vehicle_moments_ntb = vehicle.intermediate_frame_to_ntb_transform(moments)

    # Kinetic moment summation of moments not being done about CG
    # TODO: Make sure sprung inertia is about the intermediate axis
    # TODO: CoG movements
    kinetic_moments = vehicle.get_kinetic_moments(translation_accelerations_ntb) 
    
    # solving for summation of forces = m * accel
    inertial_forces = vehicle.get_inertial_forces(translation_accelerations_ntb)
    summation_forces = inertial_forces - vehicle_forces_ntb
    
    # solving for summation of moments = I * alpha
    # only rotational acceleration being considered is yaw acceleration; which is why it isnt transformed (no roll/pitch accel)
    yaw_moment = vehicle.get_yaw_moment(yaw_acceleration)
    summation_moments = np.array([0, 0, yaw_moment]) - kinetic_moments - vehicle_moments_ntb

    # LOG SOME SHITS
    [vehicle.logger.log(output_var_labels[i], x[i]) for i in range(len(x))]
    vehicle.logger.log("vehicle_accelerations_NTB", translation_accelerations_ntb)
    vehicle.logger.log("vehicle_yaw_moment", yaw_moment)
    vehicle.logger.log("vehicle_kinetic_moment", kinetic_moments)
    vehicle.logger.log("vehicle_inertial_forces", inertial_forces)
    vehicle.logger.log("vehicle_yaw_rate", yaw_rate)
    vehicle.logger.log("vehicle_x_dot", vehicle.x_dot)
    vehicle.logger.log("vehicle_y_dot", vehicle.y_dot)
    [vehicle.logger.log(name, val) for name, val in vehicle.state.items()]

    return np.array([*summation_forces, *summation_moments])

if __name__ == "__main__":
    main()