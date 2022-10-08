import numpy as np
import pandas as pd
import itertools
import engine
import multiprocessing
from tqdm import tqdm
from time import perf_counter

def ggv_generator(vehicle_params, sweep_ranges:dict, mesh:int):

    s_dot_sweep = np.linspace(sweep_ranges["velocity"][0], sweep_ranges["velocity"][1], mesh)
    body_slip_sweep = np.linspace(sweep_ranges["body_slip"][0], sweep_ranges["body_slip"][1], mesh)
    steered_angle_sweep = np.linspace(sweep_ranges["steered_angle"][0], sweep_ranges["steered_angle"][1], mesh)
    torque_request = np.linspace(sweep_ranges["torque_request"][0], sweep_ranges["torque_request"][1], mesh)
    is_left_bias = sweep_ranges["torque_request"]

    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    t1 = perf_counter()
    p = multiprocessing.Pool(multiprocessing.cpu_count())
    state_solver2 = lambda x: engine.Solver(vehicle_params).solve(engine.State(*x))
    states_product = itertools.product(body_slip_sweep, steered_angle_sweep, s_dot_sweep, torque_request, is_left_bias)
    return_list = tqdm(p.imap(state_solver2, states_product))
    p.close()

    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    log_df = pd.DataFrame.from_records(filter(None, return_list))
    log_df.to_csv("analysis/MMM.csv")
    print(f"Sweep completed in {int(perf_counter() - t1)} seconds, exported to CSV")

# def state_solver(inputs):
#     vehicle, state_inputs = inputs[:1], inputs[1:]
#     return engine.Solver(vehicle_params.EasyDriver()).solve(engine.State(*state_inputs))