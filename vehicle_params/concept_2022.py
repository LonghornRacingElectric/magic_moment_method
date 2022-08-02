import math
import numpy as np
from helpers.better_namespace import BetterNamespace

class Concept2022(BetterNamespace):
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

        # suspension tube geometry
        # Inputs (in inches, from CAD). Origin chosen to be contact patch. Initial point is outboard point.
        ### Key: 1 = FUCA; 2 = RUCA; 3 = FLCA; 4 = RLCA; 5 = Pullrod; 6 = Toe/Steering Link

        # Front:
        pt1_i = np.array([0.063, 2.138, 11.539]).reshape((3, 1))
        pt1_f = np.array([5.875, 14.641, 7.759]).reshape((3, 1))

        pt2_i = np.array([0.063, 2.138, 11.539]).reshape((3, 1))
        pt2_f = np.array([-7.375, 14.641, 6.066]).reshape((3, 1))

        pt3_i = np.array([0.066, 1.6, 4.208]).reshape((3, 1))
        pt3_f = np.array([5.875, 15.791, 2.129]).reshape((3, 1))

        pt4_i = np.array([0.066, 1.6, 4.208]).reshape((3, 1))
        pt4_f = np.array([-7.373, 15.791, 2.402]).reshape((3, 1))

        pt5_i = np.array([0.08, 3.718, 4.925]).reshape((3, 1))
        pt5_f = np.array([-5.519, 11.975, 21.429]).reshape((3, 1))

        pt6_i = np.array([2.935, 2.393, 11.049]).reshape((3, 1))
        pt6_f = np.array([4.262, 13.291, 7.523]).reshape((3, 1))

        pt_i_arr = np.concatenate((pt1_i, pt2_i, pt3_i, pt4_i, pt5_i, pt6_i), axis=1)
        pt_f_arr = np.concatenate((pt1_f, pt2_f, pt3_f, pt4_f, pt5_f, pt6_f), axis=1)

        # Create vectors that lie along tubes
        v_arr = pt_f_arr - pt_i_arr
        # Normalize vectors
        lengths = np.apply_along_axis(np.linalg.norm, 0, v_arr)
        self.front_tube_normals = v_arr / lengths

        self.front_lever_arms = pt_i_arr * 0.0254  # in to m

        # Rear
        pt1_i = np.array([-0.126, 1.549, 12.583]).reshape((3, 1))
        pt1_f = np.array([7.125, 11.492, 9.55]).reshape((3, 1))

        pt2_i = np.array([-0.126, 1.549, 12.583]).reshape((3, 1))
        pt2_f = np.array([-7.75, 11.492, 9.55]).reshape((3, 1))

        pt3_i = np.array([0.131, 1.164, 5.243]).reshape((3, 1))
        pt3_f = np.array([7.124, 13.242, 3.354]).reshape((3, 1))

        pt4_i = np.array([0.131, 1.164, 5.243]).reshape((3, 1))
        pt4_f = np.array([-5.499, 13.242, 3.613]).reshape((3, 1))

        pt5_i = np.array([-0.043, 2.38, 11.279]).reshape((3, 1))
        pt5_f = np.array([5.576, 11.847, 4.112]).reshape((3, 1))

        pt6_i = np.array([3.498, 1.026, 9.11]).reshape((3, 1))
        pt6_f = np.array([7.125, 12.089, 7.433]).reshape((3, 1))

        pt_i_arr = np.concatenate((pt1_i, pt2_i, pt3_i, pt4_i, pt5_i, pt6_i), axis=1)
        pt_f_arr = np.concatenate((pt1_f, pt2_f, pt3_f, pt4_f, pt5_f, pt6_f), axis=1)

        v_arr = pt_f_arr - pt_i_arr
        lengths = np.apply_along_axis(np.linalg.norm, 0, v_arr)
        self.rear_tube_normals = v_arr / lengths
        self.rear_lever_arms = pt_i_arr * 0.0254  # in to m

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
    