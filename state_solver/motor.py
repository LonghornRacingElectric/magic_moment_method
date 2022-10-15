import pandas as pd
import math

class Motor:
    def __init__(self, path):
        self.map = pd.read_csv(path)


    def power_input(self, torque, w, angular_velocity = True):
        """

        :param map: csv map for torque vs rpm
        :param torque: torque input
        :param w: angular velocity input
        :param angular_velocity: set to false if input for w is in units of RPM instead of angular velocity
        :return: power input given torque and angular velocity input; solves for ineffiency through map
        """
        torqueprox = round(torque)
        if angular_velocity:
            rpmprox = round(w*(60/(2*math.pi)))
        else:
            rpmprox = round(w)
            w = rpmprox*(2*math.pi)/60
        efficiency = self.map.iat[torqueprox, rpmprox]
        return torque * w / efficiency

# m = Motor()
# #
# print(Motor().power_input(110, 3000, False))

