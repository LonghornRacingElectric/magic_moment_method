from aerodynamics import Aerodynamics
from dynamics import Dynamics
import types
import numpy as np
import math
from tire import Tire


class Vehicle:
    def __init__(self):
        # TODO: Import params from files instead of declaring them as values
        self.params = types.SimpleNamespace()

        ### vehicle params
        self.params.sprung_inertia = np.array([[119.8, 1, 1], [1, 33.4, 1], [1, 1, 108.2]])  # TODO
        self.params.unsprung_inertia = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])  # TODO
        self.params.gravity = 9.81
        self.params.cg_bias = 0.55  # Position of the cg from front to rear, value from 0-1
        self.params.cg_height = 12 * 0.0254
        self.params.mass_unsprung_front = 13.5
        self.params.unsprung_front_height = 0.0254 * 8
        self.params.mass_unsprung_rear = 13.2
        self.params.unsprung_rear_height = 0.0254 * 8
        self.params.mass_sprung = 245.46
        self.params.ride_height = 0.0762  # m

        ### dynamics params
        
        # suspension
        self.params.wheelbase = 1.55
        self.params.front_track = 1.27
        self.params.rear_track = 1.17
        self.params.front_roll_stiffness = 50 * (180/math.pi) / (self.params.front_track / 2) # N/rad
        self.params.rear_roll_stiffness = 0 * (180/math.pi) / (self.params.rear_track / 2)  # N/rad
        self.params.front_wheelrate_stiffness = (.574**2) * 400 / (.0254 * .224) # N/m
        self.params.rear_wheelrate_stiffness = (.747**2) * 450 / (.0254 * .224) # N/m 
        self.params.rear_toe = 0 * math.pi/180 # rad
        self.params.front_toe = 0 # rad
        # self.front_static_camber = 0
        # self.rear_static_camber = 0
        # self.front_camber_gain = 0.6 # 0.5 deg/deg -> deg/m TODO
        # self.rear_camber_gain = 0.5 # 0.5 deg/deg -> deg/m TODO
        # self.front_roll_center_height = -.75 * .0254
        # self.rear_roll_center_height = -.5 * .0254
        # self.pitch_center_x = -2.5 * 0.0254

        # tires
        
        ### 2022
        # TODO: Make tire spring rate not a constant value
        #self.params.rear_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.rear_tire_coeff_Fy = [1.384, -0.0003117, -2.936, 668.1, 1599, 0.03877, 0.0003177, 0.6252, 7.733e-05, -0.08382,
                          -0.1171, 0.04597, 3.107, 5.41e-05, 0.04736, 0.005249, 0.0508, -0.1956]
        #self.params.rear_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.rear_tire_spring_rate = 617 * 175

        #self.params.front_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.front_tire_coeff_Fy = [1.69, 0.0004384, 2.769, 614.3, 1496, 0.01784, 0.000432, 0.7237, 0.0001746, 0.1366,
                         -0.1482, -0.06455, 10.45, 3.036e-05, 0.04111, 0.002054, 0.01834, -0.06673]
        #self.params.front_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.params.front_tire_spring_rate = 588 * 175

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

        ### dependent params
        self.params.cg_total_position = np.array([self.params.cg_bias * self.params.wheelbase, 0, self.params.cg_height])
        self.params.mass = self.params.mass_sprung + self.params.mass_unsprung_front + self.params.mass_unsprung_rear

        # Initiate component classes and vehicle parameters
        self.dynamics = Dynamics(self.params)
        self.aero = Aerodynamics(self.params)

        # set defaults, these are the prescribed MMM states
        self.state = types.SimpleNamespace()
        self.state.body_slip = 0  # RADIANS
        self.state.steered_angle = 0  # RADIANS
        self.state.x_dot = 0  # m/s
        self.state.yaw_rate = 0  # rad/s (using x_dot and body_slip, this essentially defines turn radius)

    def rotational_velocities(self, accel):
        if accel == 0:
            return np.array([0, 0 , 0])
        temp_yaw_rate = self.state.x_dot/(self.state.x_dot**2/accel)
        return np.array([0, 0, temp_yaw_rate])

    @property
    def translational_velocities(self):
        return np.array([self.state.x_dot, self.y_dot, 0])

    @property
    def y_dot(self):
        return self.state.x_dot * math.tan(self.state.body_slip)

    def get_loads(self, roll, pitch, ride_height):
        # Define aero loads
        aero_forces, aero_moments = self.aero.get_loads(self.state.x_dot, self.state.body_slip, pitch, roll,
                               ride_height)
        # Define tire loads (dynamics handles vehicle weight transfer through tire normals)
        tire_forces, tire_moments = self.dynamics.get_loads(self.state, roll, pitch, ride_height)

        return aero_forces + tire_forces, aero_moments + tire_moments

    # TODO: make these independent columns in vehicle
    def outputs(self):
        return_dict = {}
        for tire_name, tire in self.dynamics.tires.__dict__.items():
            for data_name, data in tire.outputs.__dict__.items():
                if hasattr(data, '__len__') and len(data) > 1:
                    for i in range(len(data)):
                        return_dict[tire_name + "_"+ data_name + "_" + str(i)] = data[i]
                else:
                    return_dict[tire_name + "_"+ data_name] = data
        return return_dict