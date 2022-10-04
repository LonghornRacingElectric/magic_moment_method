from helpers.better_namespace import BetterNamespace
import numpy as np

class State(BetterNamespace):
    """
    A class to represent independent car states
    """
    def __init__(self, body_slip, steered_angle, s_dot, torque_request, is_left_bias):
        """ 
        These are the prescribed MMM states
        All other vehicle tates are dependent, and solved to match conditions

        Args:
            body_slip (float): rad
            steered_angle (float): rad
            s_dot (float): m/s - vehicle total velocity (in path tangential direction)
            torque_request (float, optional): N*m. Defaults to 0.0.
        """
        self.body_slip = body_slip
        self.steered_angle = steered_angle
        self.s_dot = s_dot
        self.torque_request = torque_request
        self.is_left_bias = is_left_bias