import numpy as np
import math

class Drivetrain:
    def __init__(self, params, logger):
        self.params = params
        self.logger = logger

    def motor_power_input_and_efficiency(self, torque, angular_vel):
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

    def brake_request_to_torque(self, torque_request): # Brake request value from 0 to 1
        if torque_request > 0:
            return np.zeros(4)

        pedal_force = (self.params.max_pedal_force*torque_request)*self.params.pedal_ratio
        line_pressure = np.array([(pedal_force/self.params.master_cylinder_area)*self.params.brake_bias_ratio,
                             (pedal_force/self.params.master_cylinder_area)*(1-self.params.brake_bias_ratio)])

        torques = line_pressure*self.params.calipers_area*self.params.brake_pad_mu*self.params.rotor_radius

        return np.array([torques[0], torques[0], torques[1], torques[1]])

    def torque_bias_ratio(self, torque_on_diff, is_straight_line):
        # if on a pure straight, diff doesnt bias. Otherwise it does. BREAKAWAY TORQUE BABY
        if is_straight_line or torque_on_diff == 0:
            return np.array([0.5, 0.5])

        traction_bias = self.params.diff_fl + self.params.diff_preload/torque_on_diff

        if self.state.is_left_diff_bias:
            return np.array([traction_bias, 1 - traction_bias])
        else:
            return np.array([1 - traction_bias, traction_bias])