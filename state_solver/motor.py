import pandas as pd
import math

class Motor:
    def __init__(self, params):
        self.params = params

    def power_input_and_efficiency(self, torque, angular_vel):
        """
        :param map: csv map for torque vs rpm
        :param torque: torque input
        :param angular_vel: angular velocity input
        :return: power input and efficiency given torque and angular velocity input; solves for efficiency through map
        """
        torque_approx = round(torque)
        rpm_approx = round(angular_vel*(60/(2*math.pi)))
        efficiency = self.params.motor_map.iat[torque_approx, rpm_approx]
        power_in = torque * angular_vel / efficiency
        return power_in, efficiency