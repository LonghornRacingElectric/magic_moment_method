from aerodynamics import Aerodynamics
from suspension import Suspension
import types
import numpy as np
import math

class Vehicle:
    def __init__(self, suspension, aero):
        self.suspension = suspension
        self.aero = aero
        self.mass = 272 # kg
        self.sprung_inertia = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]]) # TODO
        self.unsprung_inertia = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]]) # TODO

        # set defaults
        self.state = types.SimpleNamespace()
        self.state.body_slip = 0 # RADIANS
        self.state.steered_angle = 0 # RADIANS
        self.state.x_dot = 0 # m/s
        self.state.yaw_rate = 0 # rad/s (using x_dot and body_slip, this essentially defines turn radius)

    @property
    def rotational_velocities(self):
        return np.array([0, 0, self.state.yaw_rate])

    @property
    def translational_velocities(self):
        return np.array([self.state.x_dot, self.y_dot, 0])

    @property
    def y_dot(self):
        return self.state.x_dot * math.tan(self.state.body_slip)

    def get_loads(self, roll, pitch, ride_height):
        forces, torques = self.aero.get_loads(self.state.x_dot, self.state.body_slip, pitch, roll, ride_height)
        # TODO: this needs help below
        forces2, torques2 = self.suspension.get_loads(**self.state.__dict__, forces = forces, torques = torques)
        return np.add(forces, forces2), np.add(torques, torques2)
