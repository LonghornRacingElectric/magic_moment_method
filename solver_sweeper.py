import numpy as np
import pandas as pd
import itertools
import multiprocessing
from tqdm import tqdm
from scipy.spatial import ConvexHull
from . import state_solver

def solver_sweeper(vehicle_params, sweep_values:dict):
    s_dot_sweep = sweep_values["velocity"]
    body_slip_sweep = sweep_values["body_slip"]
    steered_angle_sweep = sweep_values["steered_angle"]
    torque_request = sweep_values["torque_request"]
    is_left_diff_bias = sweep_values["is_left_diff_bias"]
    vehicles = [vehicle_params] # TODO: make this sweepable?

    ### ~~~ MULTIPROCESSING SOLVER BELOW ~~~ ###
    states_product = itertools.product(vehicles, body_slip_sweep, steered_angle_sweep, s_dot_sweep, torque_request, is_left_diff_bias)
    with multiprocessing.Pool(multiprocessing.cpu_count()) as p:
        results = tqdm(p.imap_unordered(solve_wrapper, states_product))
        df = pd.DataFrame(filter(None, results))

    return df

def generate_GGV(vehicle_params, sweep_ranges:dict, mesh:int):
    s_dot_sweep = np.linspace(sweep_ranges["velocity"][0], sweep_ranges["velocity"][1], mesh)
    body_slip_sweep = np.linspace(sweep_ranges["body_slip"][0], sweep_ranges["body_slip"][1], mesh)
    steered_angle_sweep = np.linspace(sweep_ranges["steered_angle"][0], sweep_ranges["steered_angle"][1], mesh)
    torque_request = np.linspace(sweep_ranges["torque_request"][0], sweep_ranges["torque_request"][1], mesh)
    sweep_values = {"velocity": s_dot_sweep, "body_slip":body_slip_sweep, "is_left_diff_bias":sweep_ranges["is_left_diff_bias"],
                    "steered_angle":steered_angle_sweep, "torque_request": torque_request}
    
    df = solver_sweeper(vehicle_params, sweep_values)
    
    ### ~~~ CONVEX HULL ~~~ ###
    hull_df = pd.DataFrame()
    for vel in df["s_dot"].unique():
        vel_df = df[df["s_dot"] == vel].reset_index()
        accel_df = vel_df[["vehicle_accelerations_NTB_1","vehicle_accelerations_NTB_0"]]
        try:
            hull_index = ConvexHull(accel_df).vertices
        except:
            hull_index = accel_df.index
        hull_df = pd.concat([hull_df, vel_df.loc[hull_index]], axis = 0)
    return hull_df

def solve_wrapper(inputs):
    vehicle, *state_inputs = inputs
    return state_solver.Solver(vehicle).solve(state_solver.State(*state_inputs))