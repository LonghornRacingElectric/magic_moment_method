import numpy as np
import pandas as pd
import engine
import vehicle_params
import multiprocessing
from tqdm import tqdm

def main():
    solver = engine.Solver(vehicle_params.EasyDriver(), generate_logs=True)

    ### ~~~ SWEEP PARAMETERS FOR MMM ~~~ ###
    # NOTE: any parameter in the vehicle_params file can be swept as well
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    peak_slip_angle = 18 * np.pi / 180 # rad
    refinement = 21

    s_dot_sweep = [5] # velocity sweep in path tangential direction (total velocity)
    body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    
    state_sweep = []
    for s_dot in s_dot_sweep:
        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                state_sweep.append(engine.State(body_slip, steered_angle, s_dot))


    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    # NOTE: On Kieran's computer with 4 cores, this improved speed by 25%
    multiprocessing_flag = False
    
    if multiprocessing_flag:
        def solver_mod(solver:engine.Solver, state, return_list):
            return_list.append(solver.solve(state))

        manager = multiprocessing.Manager()
        return_list = manager.list()
        jobs = []
        for state in state_sweep:
            p = multiprocessing.Process(target=solver_mod, args=(solver, state, return_list))
            jobs.append(p)
            p.start()

        for proc in jobs:
            proc.join()
            print(f"{int(len(return_list)/len(state_sweep)*100)}% complete")
    else:
        return_list = []

        # bar = helpers.progress_bar.ProgressBar()
        for state in tqdm(state_sweep):
            return_list.append(solver.solve(state))

            # print(f"{int(len(return_list) / len(state_sweep) * 100)}% complete")

        #     bar.update(return_list, state_sweep, solver, state)
        # bar.close()



    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    log_df = pd.DataFrame()
    for x in return_list:
        log_df = pd.concat([log_df, pd.DataFrame([x])], ignore_index=True)

    log_df.to_csv("analysis/MMM.csv")
    print("\nExport successful to CSV, MMM complete!")

if __name__ == "__main__":
    main()