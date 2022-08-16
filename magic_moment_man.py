import multiprocessing
import numpy as np
import pandas as pd
import engine
import vehicle_params


def main():
    solver = engine.Solver(vehicle_params.EasyDriver())

    ### ~~~ SWEEP PARAMETERS FOR MMM ~~~ ###
    # NOTE: any parameter in the vehicle_params file can be swept as well
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    peak_slip_angle = 18 * np.pi / 180 # rad
    refinement = 21

    s_dot_sweep = [15] # velocity sweep in path tangential direction (total velocity)
    body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    state_sweep = []
    for s_dot in s_dot_sweep:
        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                state_sweep.append(engine.State(body_slip, steered_angle, s_dot))

    log_df = sweep_solver(multiprocessing_flag=False, state_list=state_sweep, solver=solver)
    log_df.to_csv("analysis/MMM.csv")
    print("\nExport successful to CSV, MMM complete!")


def sweep_solver(multiprocessing_flag: bool, state_list: list, solver: engine.Solver):
    """_summary_

    Args:
        multiprocessing_flag (bool): _description_
        state_list (list): _description_
        solver (engine.Solver): _description_

    Returns:
        _type_: _description_
    """
    
    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    # NOTE: On Kieran's computer with 4 cores, this improved speed by 25%
    if multiprocessing_flag:
        def solver_mod(solver: engine.Solver, state, return_list: list):
            return_list.append(solver.solve(state))

        manager = multiprocessing.Manager()
        return_list = manager.list()
        jobs = []
        for state in state_list:
            proc = multiprocessing.Process(target=solver_mod, args=(solver, state, return_list))
            jobs.append(proc)
            proc.start()

        for proc in jobs:
            proc.join()
            print(f"{int(len(return_list)/len(state_list)*100)}% complete")
    else:
        return_list = []
        for state in state_list:
            return_list.append(solver.solve(state))
            print(f"{int(len(return_list)/len(state_list)*100)}% complete")

    log_df = pd.DataFrame()
    for output in return_list:
        log_df = pd.concat([log_df, pd.DataFrame([output])], ignore_index=True)            

    return log_df

if __name__ == "__main__":
    main()
