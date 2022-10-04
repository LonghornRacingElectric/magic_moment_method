import numpy as np
import pandas as pd
import itertools
import engine
import vehicle_params
import multiprocessing
import time
from tqdm import tqdm
from time import perf_counter

solver = engine.Solver(vehicle_params.EasyDriver())

def main():
    ### ~~~ SWEEP PARAMETERS FOR MMM ~~~ ###
    # NOTE: any parameter in the vehicle_params file can be swept as well
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    peak_slip_angle = 18 * np.pi / 180 # rad
    refinement = 15

    s_dot_sweep = [15] # velocity sweep in path tangential direction (total velocity)
    body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    torque_request = np.linspace(-.85, 1, refinement)
    is_left_bias = [True, False]

    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    # NOTE: Right now this actually makes it slower on most computers, cloud computing time??
    multiprocessing_flag = True
    t1 = perf_counter()
    if multiprocessing_flag:
        p = multiprocessing.Pool(multiprocessing.cpu_count())

        states_product = itertools.product(body_slip_sweep, steered_angle_sweep, s_dot_sweep, torque_request, is_left_bias)
        print(states_product)
        return_list = list(tqdm(p.imap(why, states_product)))
        p.close()
    else:
        return_list = []
        state_sweep = []
        for s_dot in s_dot_sweep:
            for body_slip in body_slip_sweep:
                for steered_angle in steered_angle_sweep:
                    for request in torque_request:
                        for bias in is_left_bias:
                            state_sweep.append(engine.State(body_slip, steered_angle, s_dot, request, bias))
        for state in tqdm(state_sweep):
            return_list.append(solver.solve(state))

            # print(f"{int(len(return_list) / len(state_sweep) * 100)}% complete")

        #     bar.update(return_list, state_sweep, solver, state)
        #bar.close()
        t2 = perf_counter()
        print("completed in {} seconds".format(t2 - t1))

    temp_df = pd.DataFrame()
    for x in return_list:
        temp_df = pd.concat([temp_df, pd.DataFrame([x])], ignore_index=True)

    #braking_df = temp_df[(abs(temp_df["front_left_tire_torque"] - temp_df["front_right_tire_torque"]) < 30) & (abs(temp_df["rear_left_tire_torque"] - temp_df["rear_right_tire_torque"]) < 30)]
    #acceleration_df = temp_df[(temp_df["slip_ratios_0"] == 0) & (temp_df["slip_ratios_1"] == 0)]
    #print(len(acceleration_df))

    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    #log_df = pd.concat(braking_df, acceleration_df)

    temp_df.to_csv("analysis/MMM.csv")
    print("\nExport successful to CSV, MMM complete!")

def why(s):
    return solver.solve(engine.State(*s))


if __name__ == "__main__":
    main()