import multiprocessing

def solver_mod(state, return_list):
    return_list.append(state*2)

def main():
    state_sweep = [1,2,3,4,5]

    manager = multiprocessing.Manager()
    return_list = manager.list()
    jobs = []

    for state in state_sweep:
        p = multiprocessing.Process(target=solver_mod, args=(state, return_list))
        jobs.append(p)
        p.start()

    for proc in jobs:
        proc.join()
        print(f"{int(len(return_list)/len(state_sweep)*100)}% complete")
        
if __name__ == "__main__":
    main()