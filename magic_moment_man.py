import numpy as np
import pandas as pd
import engine
import vehicle_params
import multiprocessing
import helpers
from tqdm import trange, tqdm

import contextlib
import sys



def main():
    solver = engine.Solver(vehicle_params.EasyDriver())

    ### ~~~ SWEEP PARAMETERS FOR MMM ~~~ ###
    # NOTE: any parameter in the vehicle_params file can be swept as well
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    peak_slip_angle = 18 * np.pi / 180 # rad
    refinement = 21

    s_dot_sweep = [5,10,15,20,25,30] # velocity sweep in path tangential direction (total velocity)
    body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
    
    state_sweep = []
    for s_dot in s_dot_sweep:
        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                state_sweep.append(engine.State(body_slip, steered_angle, s_dot))


    ### ~~~ MULTIPROCESSING BELOW ~~~ ###
    # NOTE: On Kieran's computer with 4 cores, this improved speed by 25%
    multiprocessing_flag = True
    
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

        class DummyFile(object):
            file = None

            def __init__(self, file):
                self.file = file

            def write(self, x):
                # Avoid print() second call (useless \n)
                if len(x.rstrip()) > 0:
                    tqdm.write(x, file=self.file)

        @contextlib.contextmanager
        def nostdout():
            save_stdout = sys.stdout
            sys.stdout = DummyFile(sys.stdout)
            yield
            sys.stdout = save_stdout

        def blabla():
            return_list.append(solver.solve(state))

        return_list = []
        prev = 0
        pbar = tqdm(total=100,file=sys.stdout)
        for state in state_sweep:
            with nostdout():
                blabla()
                if (int(len(return_list) / len(state_sweep) * 100)) > prev:
                    pbar.update(1)
                    prev += 1
        pbar.close()
            # pbar = tqdm(total=100)
            # for i in range(10):
            #     sleep(0.1)
            #     pbar.update(10)
            # pbar.close()
            # print(f"{int(len(return_list)/len(state_sweep)*100)}% complete")


    ### ~~~ EXPORT MMM RESULTS ~~~ ###
    log_df = pd.DataFrame()
    for x in return_list:
        log_df = pd.concat([log_df, pd.DataFrame([x])], ignore_index=True)

    log_df.to_csv("analysis/MMM.csv")
    print("\nExport successful to CSV, MMM complete!")

if __name__ == "__main__":
    main()