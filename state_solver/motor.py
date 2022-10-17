import pandas as pd
import math

class Motor:
    def __init__(self, path):
        self.map = pd.read_csv(path)





    def power_input(self, torque, w, efficiency_output = False):
        """

        :param map: csv map for torque vs rpm
        :param torque: torque input
        :param w: angular velocity input
        :param efficiency_output: if true, return motor efficiency, otherwise return power_input
        :return: power input given torque and angular velocity input; solves for ineffiency through map
        """
        torque_approx = round(torque)

        rpm_approx = round(w*(60/(2*math.pi)))

        efficiency = self.map.iat[torque_approx, rpm_approx]
        if efficiency_output:
            return efficiency
        else:
            return torque * w / efficiency

# m = Motor()
# #
# print(Motor().power_input(110, 3000, False))

