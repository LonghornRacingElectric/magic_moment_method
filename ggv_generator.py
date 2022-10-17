import math

import numpy as np
import pandas as pd
import itertools
import multiprocessing
from tqdm import tqdm
from time import perf_counter
from scipy.spatial import ConvexHull
from . import state_solver

def ggv_generator(vehicle_params, sweep_ranges:dict, mesh:int):

    s_dot_sweep = np.linspace(sweep_ranges["velocity"][0], sweep_ranges["velocity"][1], mesh)
    body_slip_sweep = np.linspace(sweep_ranges["body_slip"][0], sweep_ranges["body_slip"][1], mesh)
    steered_angle_sweep = np.linspace(sweep_ranges["steered_angle"][0], sweep_ranges["steered_angle"][1], mesh)
    torque_request = np.linspace(sweep_ranges["torque_request"][0], sweep_ranges["torque_request"][1], mesh)
    is_left_diff_bias = sweep_ranges["is_left_diff_bias"]
    vehicles = [vehicle_params]

    ### ~~~ SOLVER BELOW ~~~ ###
    states_product = itertools.product(vehicles, body_slip_sweep, steered_angle_sweep, s_dot_sweep, torque_request, is_left_diff_bias)
    t1 = perf_counter()
    with multiprocessing.Pool(multiprocessing.cpu_count()) as p:
        results = tqdm(p.imap_unordered(solve_wrapper, states_product))
        df = pd.DataFrame(filter(None, results))
    print(f"Sweep completed in {int(perf_counter() - t1)} seconds")

    ### ~~~ FILTERING ~~~ ###
    tires = ["front_left", "front_right", "rear_left", "rear_right"]
    filt_df = df[(df["roll"]*180/np.pi < 5) & (df["yaw_acceleration"] < 200) & (df["motor_angular_velocity"] < vehicle_params.max_motor_speed)]
    for tire in tires:
        filt_df = filt_df[(filt_df[f"{tire}_tire_is_saturated"] == False)
                 & (filt_df[f"{tire}_tire_tire_centric_forces_2"] > 1) & (filt_df[f"{tire}_tire_tire_centric_forces_2"] < 4000)]
    
    # Filter if power limit hit
    motor = state_solver.Motor("engine/magic_moment_method/vehicle_params/Eff228.csv")

    filt_df[['power_input', 'motor_efficiency']] = filt_df.apply(lambda x: motor.power_input_and_efficiency(
            x['motor_torque'], x['motor_angular_velocity']), axis=1, result_type="expand")
    # filt_df['motor_efficiency'] = filt_df.apply(lambda x: motor.power_input(x['motor_torque'], x['motor_angular_velocity'], efficiency_output=True),
    #                                        axis=1)
    filt_df = filt_df[filt_df["power_input"] < 80000]
    filt_df['motor_current'] = filt_df['motor_torque'] / (220 / 350) # TODO: move this to Motor
    filt_df['motor_voltage'] = filt_df['power_input'] / filt_df['motor_current']

    ### ~~~ CONVEX HULL ~~~ ###
    hull_df = pd.DataFrame()
    for vel in filt_df["s_dot"].unique():
        vel_df = filt_df[filt_df["s_dot"] == vel].reset_index()
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