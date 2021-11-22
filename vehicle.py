from aerodynamics import Aerodynamics
from dynamics import Dynamics
from better_namespace import BetterNamespace
import numpy as np
from math import cos, sin


class Vehicle:
    def __init__(self, params):
        self.params = params

        # Initiate component classes and vehicle parameters
        self.dynamics = Dynamics(self.params)
        self.aero = Aerodynamics(self.params)

        # set defaults, these are the prescribed MMM states
        self.state = BetterNamespace()
        self.state.body_slip = 0  # rad
        self.state.steered_angle = 0  # rad
        self.state.s_dot = 0  # m/s # Intermediate Frame
        
        # Intermediate states calculated along the way, saved for analysis :)
        self.outputs = BetterNamespace()
        self.outputs.dynamics = self.dynamics.outputs
        self.outputs.aero = self.aero.outputs
        self.outputs.front_left_tire = self.dynamics.tires.front_left.outputs
        self.outputs.front_right_tire = self.dynamics.tires.front_right.outputs
        self.outputs.rear_left_tire = self.dynamics.tires.rear_left.outputs
        self.outputs.rear_right_tire = self.dynamics.tires.rear_right.outputs
        
        self.outputs.vehicle = BetterNamespace()
        self.outputs.vehicle.yaw_rate = None # rad/s  ## Using velocity and normal acceleration
        self.outputs.vehicle.turn_radius = None # m  ## Using velocity and normal acceleration
        self.outputs.vehicle.accelerations_NTB = None # m/s^2

    # normal to path acceleration (lateral accel) passed in; yaw rate dependent state on this as described by below equations
    def set_turn_radius_and_yaw_velocity(self, lateral_accel):
        # no slip condition; yaw rate guaranteed by acceleration and velocity
        if lateral_accel == 0:
            self.outputs.vehicle.turn_radius = 0
            self.outputs.vehicle.yaw_rate = 0
        else:
            # v^2/r = a; w*r = v; w = v/r = v/(v^2/a); alpha = a/r
            self.outputs.vehicle.turn_radius = self.state.s_dot ** 2 / lateral_accel
            self.outputs.vehicle.yaw_rate = self.state.s_dot / self.outputs.vehicle.turn_radius

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
        # only use normal acceleration for yaw velocity & turn radius calc!
        self.set_turn_radius_and_yaw_velocity(lateral_accel)
        
        # Define aero loads
        aero_forces, aero_moments = self.aero.get_loads(self.x_dot, self.state.body_slip, pitch, roll,
                               ride_height)
        
        # Define tire loads (dynamics handles vehicle weight transfer through tire normals)
        tire_forces, tire_moments = self.dynamics.get_loads(self.translational_velocities_IMF, self.outputs.vehicle.yaw_rate,
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