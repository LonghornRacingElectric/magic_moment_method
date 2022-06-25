import numpy as np
import pandas as pd
import engine
import vehicle_params


def main():
    # These are the output variables being solved for to match the prescribed states!
    initial_guess = {"ride_height": 0.0762, "x_double_dot": 0, "y_double_dot": 0, "yaw_acceleration":0,
                        "roll": 0, "pitch": 0}
    
    solver = engine.Solver(vehicle_params.EasyDriver(), initial_guess)
    log_df = pd.DataFrame()

    # sweep parameters for MMM; any parameter in the vehicle_params file can be swept as well.
    # total velocity in body slip direction
    for s_dot in [15]:
        
        # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
        peak_slip_angle = 18 * np.pi / 180 # rad 
        
        for body_slip in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
            for steered_angle in np.linspace(-peak_slip_angle, peak_slip_angle, 21):
                
                output_dict = solver.solve(engine.State(body_slip, steered_angle, s_dot))
                
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