from aerodynamics import Aerodynamics
from dynamics import Dynamics
from better_namespace import BetterNamespace
import numpy as np
import math
from math import cos, sin


class Vehicle:
    def __init__(self):
        # TODO: Import params from files instead of declaring them as values
        self.params = BetterNamespace()

        ### vehicle params
        self.params.sprung_inertia = np.array([[119.8, 0, 0], [0, 33.4, 0], [0, 0, 108.2]])  # kg*m^2
        self.params.gravity = 9.81 # m/s^2
        self.params.cg_bias = 0.53  # Position of the cg from front to rear, value from 0->1
        self.params.cg_height = 11.09 * 0.0254 # m 
        self.params.mass_sprung = 245.46 # kg

        ### dynamics params
        
        # suspension
        self.params.front_wheelrate_stiffness = (.574**2) * 400 / (.0254 * .224) # N/m
        self.params.rear_wheelrate_stiffness = (.747**2) * 450 / (.0254 * .224) # N/m
        self.params.wheelbase = 61 * 0.0254 # m
        self.params.front_track = 1.27 # m
        self.params.rear_track = 1.17 # m
        self.params.ride_height = 0.0762  # m
        
        # These are ARB stiffnesses!
        # 143 is Lorraine!! 50 is using Lorraine heave stiffness for balance
        self.params.front_roll_stiffness = 50 * (180/math.pi) / (self.params.front_track / 2) # N/rad 
        self.params.rear_roll_stiffness = 0 * (180/math.pi) / (self.params.rear_track / 2)  # N/rad
        
        self.params.front_wheelrate_stiffness = (.574**2) * 400 / (.0254 * .224) # N/m
        self.params.rear_wheelrate_stiffness = (.747**2) * 450 / (.0254 * .224) # N/m 
        self.params.rear_toe = 0 * math.pi/180 # rad
        self.params.front_toe = 0 # rad
        self.params.front_static_camber = -1 * (np.pi / 180)
        self.params.rear_static_camber = -1 * (np.pi / 180)
        self.params.front_camber_gain = 0.5 # 0.5 deg/deg -> deg/m TODO
        self.params.rear_camber_gain = 0.5 # 0.5 deg/deg -> deg/m TODO
        self.params.front_KPI = 4.2 * (np.pi / 180) # 5 deg -> radians; 2021 number
        self.params.rear_KPI = 3 * (np.pi / 180) # 3 deg -> radians; 2021 number
        self.params.front_caster = 2 * (np.pi / 180) # 1 deg -> radians
        self.params.rear_caster = 2 * (np.pi / 180)  # 2 deg -> radians
        # self.front_roll_center_height = -.75 * .0254
        # self.rear_roll_center_height = -.5 * .0254
        # self.pitch_center_x = -2.5 * 0.0254

        # unsprung & tires
        self.params.mass_unsprung_front = 13.5 # kg
        self.params.mass_unsprung_rear = 13.2 # kg
        
        ### 2022
        # TODO: Make tire spring rate not a constant value
        #self.params.rear_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.rear_tire_coeff_Fy = [1.384, -0.0003117, -2.936, 668.1, 1599, 0.03877, 0.0003177, 0.6252, 7.733e-05, -0.08382,
                          -0.1171, 0.04597, 3.107, 5.41e-05, 0.04736, 0.005249, 0.0508, -0.1956]
        #self.params.rear_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.rear_tire_spring_rate = 617 * 175 # N/m

        #self.params.front_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.front_tire_coeff_Fy = [1.69, 0.0004384, 2.769, 614.3, 1496, 0.01784, 0.000432, 0.7237, 0.0001746, 0.1366,
                         -0.1482, -0.06455, 10.45, 3.036e-05, 0.04111, 0.002054, 0.01834, -0.06673]
        #self.params.front_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.front_tire_spring_rate = 588 * 175 # N/m

        ### LORRAINE
        #self.params.rear_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        # self.params.rear_tire_coeff_Fy = [1.384, -0.0003117, -2.936, 668.1, 1599, 0.03877, 0.0003177, 0.6252, 7.733e-05, -0.08382,
        #                   -0.1171, 0.04597, 3.107, 5.41e-05, 0.04736, 0.005249, 0.0508, -0.1956]
        #self.params.rear_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        # self.params.rear_tire_spring_rate = 617 * 175

        #self.params.front_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        # self.params.front_tire_coeff_Fy = [1.311, -0.0003557, -2.586, 550.1, 1403, 0.00956, 3.087e-07,0.0004554, 0.0003098, 0.1428,
        #                  -0.1516, -0.1516, 0.304, 2.038e-05, 0.02862, 0.001671, -71.72, -281.9]
        #self.params.front_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        # TODO: Double check below spring rate
        # self.params.front_tire_spring_rate = 551 * 175

        ### aerodynamics params
        self.params.ClA_tot = 3.955
        self.params.CdA0 = 0.7155
        self.params.CdA_full = 1.512
        self.params.CsA_tot = 33.91

        ### vehicle dependent params
        self.params.cg_total_position = np.array([self.params.cg_bias * self.params.wheelbase, 0, self.params.cg_height])
        self.params.mass = self.params.mass_sprung + self.params.mass_unsprung_front + self.params.mass_unsprung_rear

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