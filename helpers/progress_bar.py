import contextlib
import sys
from tqdm import tqdm


class DummyFile():
    file = None

    def __init__(self, file):
        self.file = file

    def write(self, x):
        # Avoid print() second call (useless \n)
        if len(x.rstrip()) > 0:
            tqdm.write(x, file=self.file)


class ProgressBar():

    def __init__(self):
        self.prev = 0
        self.pbar = tqdm(total=100, file=sys.stdout)

    @contextlib.contextmanager
    def nostdout(self):
        save_stdout = sys.stdout
        sys.stdout = DummyFile(sys.stdout)
        yield
        sys.stdout = save_stdout

    def abc(self):
        print("A123")

    def update(self, return_list, state_sweep, solver, state):

        with self.nostdout():
            return_list.append(solver.solve(state))


            if (int(len(return_list) / len(state_sweep) * 100)) > self.prev:
                self.pbar.update(1)
                self.prev += 1

    def close(self):
        self.pbar.close()

# asdf = ProgressBar()
# for i in range(100):
#     asdf.blabla(None,None,None)