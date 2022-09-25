import numpy as np
import pandas as pd
import engine
import vehicle_params
import multiprocessing
import time
from tqdm import tqdm


import itertools
from multiprocessing import Pool
from time import perf_counter

solver = engine.Solver(vehicle_params.EasyDriver())


def main():
    ### ~~~ SWEEP PARAMETERS FOR MMM ~~~ ###
    # NOTE: any parameter in the vehicle_params file can be swept as well
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    peak_slip_angle = 18 * np.pi / 180  # rad
    refinement = 25


    s_dot_sweep = [5]  # velocity sweep in path tangential direction (total velocity)
    body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    torque_request_sweep = [0]


    # old computes in 120 seconds
    # new computes in 3 seconds

    t1 = perf_counter()

    p = Pool(multiprocessing.cpu_count())

    multi = False
    if multi:
        states_product = itertools.product(body_slip_sweep, steered_angle_sweep, s_dot_sweep, torque_request_sweep)
        return_list = list(tqdm(p.imap(why, states_product)))
    else:
        return_list = []
        state_sweep = []
        for s_dot in s_dot_sweep:
            for body_slip in body_slip_sweep:
                for steered_angle in steered_angle_sweep:
                    state_sweep.append(engine.State(body_slip, steered_angle, s_dot))
        for state in tqdm(state_sweep):
            return_list.append(solver.solve(state))

    t2 = perf_counter()
    print("completed in {} seconds".format(t2 - t1))

    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    log_df = pd.DataFrame()
    for x in return_list:
        log_df = pd.concat([log_df, pd.DataFrame([x])], ignore_index=True)
    if multi:
        log_df.to_csv("analysis/MMM_fast.csv")
    else:
        log_df.to_csv("analysis/MMM.csv")
    print("\nExport successful to CSV, MMM complete!")


def why(s):
    return solver.solve(engine.State(*s))




if __name__ == "__main__":
    main()