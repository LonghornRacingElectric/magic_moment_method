import numpy as np
import pandas as pd
import engine
import vehicle_params
import multiprocessing
import time

solver = engine.Solver(vehicle_params.EasyDriver())

def main():
    ### ~~~ SWEEP PARAMETERS FOR MMM ~~~ ###
    # NOTE: any parameter in the vehicle_params file can be swept as well
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    peak_slip_angle = 18 * np.pi / 180 # rad
    refinement = 21

    s_dot_sweep = [13] # velocity sweep in path tangential direction (total velocity)
    body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    
    state_sweep = []
    for s_dot in s_dot_sweep:
        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                state_sweep.append(engine.State(body_slip, steered_angle, s_dot, [5, 5, 5, 5]))


    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    # NOTE: Right now this actually makes it slower on most computers, cloud computing time??
    multiprocessing_flag = False
    
    if multiprocessing_flag:
        manager = multiprocessing.Manager()
        return_list = manager.list()
        jobs = []
        for state in state_sweep:
            p = multiprocessing.Process(target=solver_mod, args=(state.values(), return_list))
            jobs.append(p)
            p.start()
            while len(jobs) - len(return_list) > 2:
                time.sleep(0.001)
            print(f"{int(len(return_list)/len(state_sweep)*100)}% complete")

        for proc in jobs:
            proc.join()

    else:
        return_list = []
        for state in state_sweep:
            x = solver.solve(state)
            if x is not None:
                return_list.append(x)
            print(f"{int(len(return_list)/len(state_sweep)*100)}% complete")

    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    log_df = pd.DataFrame()
    for x in return_list:
        log_df = pd.concat([log_df, pd.DataFrame([x])], ignore_index=True)

    log_df.to_csv("analysis/MMM.csv")
    print("\nExport successful to CSV, MMM complete!")

def solver_mod(state_list, return_list):
    x = solver.solve(engine.State(*state_list, [5, 5, 5, 5]))
    if x is not None:
        return_list.append(x)

if __name__ == "__main__":
    main()