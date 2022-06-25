import numpy as np
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
import warnings
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

import engine
import vehicle_params


def main():
    # These are the output variables being solved for to match the prescribed states!
    output_var_labels = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
    initial_guess = [0.0762, 0, 0, 0, 0, 0]
    vehicle = engine.Vehicle(vehicle_params.EasyDriver())
    specific_residual_func = lambda x: engine.Residuals.DOF6_motion_residuals(x, vehicle, output_var_labels)
    log_df = pd.DataFrame()

    # sweep parameters for MMM; any parameter in the vehicle_params file can be swept as well.
    # total velocity in body slip direction
    for s_dot in [15]:
        
        # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
        peak_slip_angle = 18 * np.pi / 180 # rad 
        
        for body_slip in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
            for steered_angle in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
                vehicle.state = engine.State(body_slip, steered_angle, s_dot)
                
                # solve for unique output variable set to match the prescribed states, with an initial guess of outputs
                josie_solver(specific_residual_func, initial_guess)
                
                 # ~~~ Handles Post Processing of Solution ~~~ #
                output_dict = copy(vehicle.logger.return_log())
                
                # see if point is saturated (i.e. all 4 tires slip angles are saturated)
                # saturation will yield to useless data point since it will wrap back around with less acceleration
                # if not saturated, the point will be saved
                # TODO: Should 1 tire even be allowed to lift???
                if not output_dict["dynamics_tires_saturated"] and not output_dict["dynamics_two_tires_lifting"]:
                    log_df = pd.concat([log_df, pd.DataFrame([output_dict])], ignore_index=True)
    
    # add solved outputs to CSV file along with intermediate logged values
    log_df.to_csv("analysis/MMM.csv")

    print("\nExport successful to CSV, MMM complete!")

if __name__ == "__main__":
    main()