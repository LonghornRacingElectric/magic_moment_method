from aerodynamics import Aerodynamics
from suspension import Suspension
import types
import numpy as np

class Vehicle:
    def __init__(self, suspension, aero):
        self.suspension = suspension
        self.aero = aero
        self.state = types.SimpleNamespace()

    def get_loads(self, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch, ride_height):
        forces, moments = self.aero.get_loads(self.state.x_dot, self.state.body_slip, pitch, roll, ride_height)
        # TODO: this needs help below
        forces2, moments2 = self.suspension.get_loads()
        return np.add(forces, forces2), np.add(moments, moments2)
