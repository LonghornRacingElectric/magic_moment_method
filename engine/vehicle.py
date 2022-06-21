from engine.aerodynamics import Aerodynamics
from engine.suspension import Suspension
from helpers.better_namespace import BetterNamespace
import numpy as np
from math import cos, sin


class Vehicle:
    def __init__(self, params, state):
        self.params = params

        # Initiate component classes and vehicle parameters
        self.suspension = Suspension(self.params)
        self.aero = Aerodynamics(self.params)

        # set defaults, these are the prescribed MMM states
        self.state = state
        self.state.body_slip = 0  # rad
        self.state.steered_angle = 0  # rad
        self.state.s_dot = 0  # m/s # Intermediate Frame
        
        # Intermediate states calculated along the way, saved for analysis :)
        self.outputs = BetterNamespace()
        self.outputs.dynamics = self.suspension.outputs
        self.outputs.aero = self.aero.outputs
        self.outputs.front_left_tire = self.suspension.tires.front_left.outputs
        self.outputs.front_right_tire = self.suspension.tires.front_right.outputs
        self.outputs.rear_left_tire = self.suspension.tires.rear_left.outputs
        self.outputs.rear_right_tire = self.suspension.tires.rear_right.outputs
        
        # NOTE: these outputs are for LOGGING PURPOSES ONLY. Dont use in the code
        self.outputs.vehicle = BetterNamespace()
        self.outputs.vehicle.yaw_rate = None # rad/s  ## Using velocity and normal acceleration
        self.outputs.vehicle.turn_radius = None # m  ## Using velocity and normal acceleration
        self.outputs.vehicle.accelerations_NTB = None # m/s^2 
        self.outputs.vehicle.yaw_moment = None # N*m
        self.outputs.vehicle.kinetic_moments = None # N*m
        self.outputs.vehicle.inertial_forces = None # N
    
    def get_yaw_moment(self, yaw_acceleration):
        self.outputs.vehicle.yaw_moment = np.dot(self.params.sprung_inertia, [0, 0, yaw_acceleration])[2]
        return self.outputs.vehicle.yaw_moment

    def get_kinetic_moments(self, linear_accelerations):
        # TODO: CoG movements
        cg_relative_ntb = np.array([0, 0, self.params.cg_total_position[2]])
        self.outputs.vehicle.kinetic_moment = np.cross(self.params.mass * linear_accelerations, cg_relative_ntb)
        return self.outputs.vehicle.kinetic_moment
    
    def get_inertial_forces(self, linear_accelerations):
        self.outputs.vehicle.inertial_forces =  self.params.mass * linear_accelerations
        return self.outputs.vehicle.inertial_forces
    
    def log_linear_accelerations_ntb(self, linear_accelerations_ntb):
        self.outputs.vehicle.accelerations_NTB = linear_accelerations_ntb

    # normal to path acceleration (lateral accel) passed in; yaw rate dependent state on this as described by below equations
    # NOTE: turn radius and yaw rate being added to outputs for logging
    def get_yaw_rate(self, lateral_accel):
        # no slip condition; yaw rate guaranteed by acceleration and velocity
        if lateral_accel == 0:
            self.outputs.vehicle.turn_radius = 0
            self.outputs.vehicle.yaw_rate = 0
        else:
            # v^2/r = a; w*r = v; w = v/r = v/(v^2/a); alpha = a/r
            self.outputs.vehicle.turn_radius = self.state.s_dot ** 2 / lateral_accel
            self.outputs.vehicle.yaw_rate = self.state.s_dot / self.outputs.vehicle.turn_radius
        return self.outputs.vehicle.yaw_rate

    # Intermediate Frame rotational velocities
    @property
    def rotational_velocities_IMF(self):
        return np.array([0, 0, self.outputs.vehicle.yaw_rate])

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

    def get_loads(self, roll, pitch, ride_height, lateral_accel):
        # NOTE: only use normal acceleration for yaw velocity & turn radius calc!
        # TODO: add more clarification on this importance
        yaw_rate = self.get_yaw_rate(lateral_accel)
        
        # Define aero loads
        aero_forces, aero_moments = self.aero.get_loads(self.x_dot, self.state.body_slip, pitch, roll,
                               ride_height)
        
        # Define tire loads (dynamics handles vehicle weight transfer through tire normals)
        tire_forces, tire_moments = self.suspension.get_loads(self.translational_velocities_IMF, yaw_rate,
                                                            self.state.steered_angle, roll, pitch, ride_height)

        return aero_forces + tire_forces, aero_moments + tire_moments

    # grab all input states and dependent (solved) states for output analysis
    def output_log(self):
        return_dict = {}
        for output_name, output in self.outputs.items():
            for data_name, data in output.items():
                if hasattr(data, '__len__') and len(data) > 1:
                    for i in range(len(data)):
                        return_dict[output_name + "_"+ data_name + "_" + str(i)] = data[i]
                else:
                    return_dict[output_name + "_"+ data_name] = data
        return_dict.update(dict(self.state.items()))
        return return_dict