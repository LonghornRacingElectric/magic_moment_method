from aerodynamics import Aerodynamics
from suspension import Suspension
import types
import numpy as np
import math


class Vehicle:
    def __init__(self):
        # Params
        self.params = types.SimpleNamespace()
        self.params.sprung_inertia = np.array([[100, 1, 1], [1, 120, 1], [1, 1, 70]])  # TODO
        self.params.unsprung_inertia = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])  # TODO
        self.params.gravity = 9.81
        self.params.cg_bias = 0.6  # Position of the cg from front to rear, value from 0-1
        self.params.wheelbase = 1.55
        self.params.cg_total_position = np.array([self.params.cg_bias * self.params.wheelbase, 0, 0.0254 * 10])
        self.params.mass_unsprung_front = 13.5
        self.params.unsprung_front_height = 0.0254 * 8
        self.params.mass_unsprung_rear = 13.2
        self.params.unsprung_rear_height = 0.0254 * 8
        self.params.mass_sprung = 245.46
        self.params.mass = self.params.mass_sprung + self.params.mass_unsprung_front + self.params.mass_unsprung_rear
        self.params.ClA_tot = 3.955
        self.params.CdA0 = 0.7155
        self.params.CdA_full = 1.512
        self.params.CsA_tot = 33.91

        # Initiate component classes and vehicle parameters
        self.suspension = Suspension(self.params)
        self.aero = Aerodynamics(self.params)

        # set defaults, these are the prescribed MMM states
        self.state = types.SimpleNamespace()
        self.state.body_slip = 0  # RADIANS
        self.state.steered_angle = 0  # RADIANS
        self.state.x_dot = 0  # m/s
        self.state.yaw_rate = 0  # rad/s (using x_dot and body_slip, this essentially defines turn radius)

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
        forces, torques = np.zeros(3), np.zeros(3)

        # Define aero loads
        aero_forces, aero_moments = 0,0 #self.aero.get_loads(self.state.x_dot, self.state.body_slip, pitch, roll,
                                # ride_height)
        # Define suspension loads (suspension handles vehicle weight through tire normals)
        # TODO: Don't pass both bodyslip and ydot
        suspension_forces, suspension_moments = self.suspension.get_loads(*self.state.__dict__.values(), self.y_dot, roll, pitch, ride_height)

        forces, torques = aero_forces + suspension_forces, aero_moments + suspension_moments
        return forces, torques