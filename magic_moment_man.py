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
    torque_request = np.linspace(-1, 1, refinement)
    is_left_bias = [True, False]

    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    t1 = perf_counter()
    p = multiprocessing.Pool(multiprocessing.cpu_count())
    states_product = itertools.product(body_slip_sweep, steered_angle_sweep, s_dot_sweep, torque_request, is_left_bias)
    return_list = tqdm(p.imap(state_solver, states_product))
    p.close()

    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    log_df = pd.DataFrame.from_records(filter(None, return_list))
    log_df.to_csv("analysis/MMM.csv")
    print(f"Sweep completed in {int(perf_counter() - t1)} seconds, exported to CSV")

def state_solver(state_inputs):
    return solver.solve(engine.State(*state_inputs))

if __name__ == "__main__":
    main()