import math
import numpy as np
from vehicle_params import BaseVehicle

class Concept2022(BaseVehicle):
    def __init__(self):
        super().__init__()

        ### vehicle params
        self.sprung_inertia = np.array([[119.8, 0, 0], [0, 33.4, 0], [0, 0, 108.2]])  # kg*m^2
        self.gravity = 9.81 # m/s^2
        self.cg_bias = 0.53  # Position of the cg from front to rear, value from 0->1
        self.cg_height = 11.09 * 0.0254 # m 
        self.mass_sprung = 220 # kg

        ### dynamics params
        
        # suspension
        self.front_wheelrate_stiffness = 30000 #(.574**2) * 400 / (.0254 * .224) # N/m # TODO for 2022
        self.rear_wheelrate_stiffness = 35000 #(.747**2) * 450 / (.0254 * .224) # N/m # TODO for 2022
        self.wheelbase = 65 * 0.0254 # m
        self.front_track = 1.27 # m
        self.rear_track = 1.17 # m
        self.ride_height = 0.0762 # m
        
        # These are ARB stiffnesses! 50 is using Lorraine heave stiffness for balance
        self.front_roll_stiffness = 4032 / (self.front_track / 2) # N/rad 
        self.rear_roll_stiffness = 0 * (180/math.pi) / (self.rear_track / 2)  # N/rad
        
        self.front_wheelrate_stiffness = 20000 #(.574**2) * 400 / (.0254 * .224) # N/m
        self.rear_wheelrate_stiffness = 40000 #(.747**2) * 450 / (.0254 * .224) # N/m 
        self.rear_toe = 0 * math.pi/180 # rad
        self.front_toe = 0 # rad
        self.front_static_camber = -1 * (np.pi / 180) 
        self.rear_static_camber = -1 * (np.pi / 180)
        self.front_camber_gain = 0.5 # 0.5 deg/deg -> deg/m TODO
        self.rear_camber_gain = 1 # 0.5 deg/deg -> deg/m TODO
        self.front_KPI = 4.2 * (np.pi / 180) # 5 deg -> radians; 2021 number
        self.rear_KPI = 3 * (np.pi / 180) # 3 deg -> radians; 2021 number
        self.front_caster = 2 * (np.pi / 180) # 1 deg -> radians
        self.rear_caster = 2 * (np.pi / 180)  # 2 deg -> radians
        # front_roll_center_height = -.75 * .0254
        # rear_roll_center_height = -.5 * .0254
        # pitch_center_x = -2.5 * 0.0254

        # unsprung & tires
        self.mass_unsprung_front = 13.5 # kg
        self.mass_unsprung_rear = 13.2 # kg
        
        ### 2022
        # TODO: Make tire spring rate not a constant value
        #self.rear_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.rear_tire_coeff_Fy = [1.384, -0.0003117, -2.936, 668.1, 1599, 0.03877, 0.0003177, 0.6252, 7.733e-05, -0.08382,
                        -0.1171, 0.04597, 3.107, 5.41e-05, 0.04736, 0.005249, 0.0508, -0.1956]
        #self.rear_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.rear_tire_spring_rate = 617 * 175 # N/m

        #self.front_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.front_tire_coeff_Fy = [1.69, 0.0004384, 2.769, 614.3, 1496, 0.01784, 0.000432, 0.7237, 0.0001746, 0.1366,
                        -0.1482, -0.06455, 10.45, 3.036e-05, 0.04111, 0.002054, 0.01834, -0.06673]
        #self.front_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.front_tire_spring_rate = 588 * 175 # N/m

        ### aerodynamics params
        self.ClA_tot = 3.955
        self.CdA0 = 0.7155
        self.CdA_full = 1.512
        self.CsA_tot = 33.91

        ### vehicle dependent params
        self.cg_total_position = np.array([self.cg_bias * self.wheelbase, 0, self.cg_height])
        self.mass = self.mass_sprung + self.mass_unsprung_front + self.mass_unsprung_rear
    