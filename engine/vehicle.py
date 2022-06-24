from engine.aerodynamics import Aerodynamics
from engine.suspension import Suspension
from engine.state import State
from engine.logger import Logger
from helpers.better_namespace import BetterNamespace
import numpy as np
from math import cos, sin


class Vehicle:
    # state: These are the prescibed MMM States
    # logger: dependent states & values calculated in solution, saved for post-analysis
    # params: static car parameters
    # aero: handles aerodynamic induced forces
    # suspension: handles suspension induced forces
    def __init__(self, params, state = State(), logger = Logger()):
        self.state = state
        self.logger = logger
        self.params = params
        
        [self.logger.log(name, val) for name, val in state.items()]
        
        self.suspension = Suspension(self.params, self.logger)
        self.aero = Aerodynamics(self.params, self.logger)
    
    def get_yaw_moment(self, yaw_acceleration):       
        return np.dot(self.params.sprung_inertia, [0, 0, yaw_acceleration])[2]

    # TODO: CoG movements from pitch & roll, which will affect this term
    def get_kinetic_moments(self, linear_accelerations):
        cg_relative_ntb = np.array([0, 0, self.params.cg_total_position[2]])
        return np.cross(self.params.mass * linear_accelerations, cg_relative_ntb)
    
    def get_inertial_forces(self, linear_accelerations):       
        return self.params.mass * linear_accelerations

    # normal to path acceleration (lateral accel) passed in; yaw rate dependent state on this as described by below equations
    # NOTE: turn radius being added to outputs for logging
    def get_yaw_rate(self, lateral_accel):
        # no slip condition; yaw rate guaranteed by acceleration and velocity
        if lateral_accel == 0:
            turn_radius = 0
            yaw_rate = 0
        else:
            # v^2/r = a; w*r = v; w = v/r = v/(v^2/a); alpha = a/r
            turn_radius = self.state.s_dot ** 2 / lateral_accel
            yaw_rate = self.state.s_dot / turn_radius
            
        self.logger.log("vehicle_turn_radius", turn_radius)

        return yaw_rate

    # Intermediate Frame translational velocities
    @property
    def translational_velocities_IMF(self):
        return np.array([self.x_dot, self.y_dot, 0])
    
    # Transform input array to NTB from Intermediate Frame using body slip
    def intermediate_frame_to_ntb_transform(self, intermediate_frame_vector):
        transformation_matrix = np.linalg.inv(np.array([[cos(self.state.body_slip), -sin(self.state.body_slip), 0],
                                          [sin(self.state.body_slip), cos(self.state.body_slip), 0],
                                          [0, 0, 1]]))
        return np.dot(transformation_matrix, intermediate_frame_vector)       
    
    @property
    def x_dot(self):
        return self.state.s_dot * cos(self.state.body_slip)
    
    @property
    def y_dot(self):
        return self.state.s_dot * sin(self.state.body_slip)

    def get_loads(self, roll, pitch, ride_height, yaw_rate):
        # Define aero loads
        aero_forces, aero_moments = self.aero.get_loads(self.x_dot, self.state.body_slip, pitch, roll,
                               ride_height)
        
        # Define tire loads (dynamics handles vehicle weight transfer through tire normals)
        tire_forces, tire_moments = self.suspension.get_loads(self.translational_velocities_IMF, yaw_rate,
                                                            self.state.steered_angle, roll, pitch, ride_height)

        return aero_forces + tire_forces, aero_moments + tire_moments