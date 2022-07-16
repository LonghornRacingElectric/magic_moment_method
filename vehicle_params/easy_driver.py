import math
import numpy as np
from helpers.better_namespace import BetterNamespace

# ALL STATIC PARAMETERS GO HERE (or parameters assumed to be static)
class EasyDriver(BetterNamespace):
    def __init__(self):
        super().__init__()

        ### vehicle params ###
        
        self.sprung_inertia = np.array([[119.8, 0, 0], [0, 33.4, 0], [0, 0, 108.2]])  # kg*m^2 # TODO is this correct? check solidworks
        self.gravity = 9.81 # m/s^2
        # TODO: make cg bias of car and driver, so driver mass can be swept to see its affect on car performance
        self.cg_bias = (1 - .456) #.456)  # % rear/total, value from 0->1
        self.cg_left = 0.495 # % left/total, value from 0->1 # TODO: not behaving as expected
        self.cg_height = 11.7 * (0.0254) # m
        self.wheelbase = 65 * (0.0254) # m
        self.front_track = 48 * (0.0254) # m
        self.rear_track = 46 * (0.0254) # m
        
        self.mass_unsprung_front = 23  * (0.4359)  # kg
        self.mass_unsprung_rear = 22 * (0.4359) # kg
        self.driver_mass = 150 * (0.4359) # kg
        self.mass_sprung = 552 * (0.4359) - 2 * self.mass_unsprung_front - 2 * self.mass_unsprung_rear + self.driver_mass # kg



        ### suspension params ###
        
        # ~~~ Stiffnesses ~~~ #
        self.front_spring_springrate = 550 * (4.448 / 0.0254) # N/m
        self.rear_spring_springrate = 650 * (4.448 / 0.0254) # N/m
        # TODO: make MRs not constant values (add nonlinear solver to this portion)
        self.front_motion_ratio = 1.92 # m/m
        self.rear_motion_ratio = 1.45 # m/m
        self.antidive = 0.2 # % 0->1 (NOTE: Milliken pg. 618) 
        # TODO: verify matches CAD right now
        # NOTE: not being used ATM, need to figure out how it applies to slip angle drag
        
        # NOTE: Front ARB stiffness set such that roll stiffness F/R makes 50/50 bias on Easy Driver
        self.front_arb_stiffness = 5000 * self.front_track**2 / 2 # N/rad
        self.rear_arb_stiffness = 0#50000  * self.rear_track**2 / 2 # N/rad
        
        # ~~~ Linkages & HDPTs ~~~ #
        self.rear_toe = 1 * (math.pi / 180) # rad  # TODO: not correct to car
        self.front_toe = 0 * (math.pi / 180) # rad # TODO: not correct to car
        self.front_static_camber = -1 * (np.pi / 180) # rad # TODO: not correct to car
        self.rear_static_camber = -1 * (np.pi / 180) # rad # TODO: not correct to car
        self.front_camber_gain = 0.7 # 0.5 deg/deg -> deg/m # TODO: IMPLEMENT
        self.rear_camber_gain = 0.7 # 0.5 deg/deg -> deg/m # TODO: IMPLEMENT
        self.front_KPI = 4.2 * (np.pi / 180) # rad
        self.rear_KPI = 3 * (np.pi / 180) # rad
        self.front_caster = 2 * (np.pi / 180) # rad
        self.rear_caster = 2 * (np.pi / 180)  # rad
        self.rear_toe_gain = 0.18 * (np.pi / 180) # rad/rad # TODO: not correct to car; also TODO implement
        self.front_roll_center_height = -.75 * .0254 # m  # TODO: not correct to car; also heavily NONLINEAR
        self.rear_roll_center_height = -.5 * .0254 # m  # TODO: not correct to car; also heavily NONLINEAR
        
        # ~~~ Tires & Pacejka ~~~ #
        # NOTE: These lateral fits all assumed slip angle was in DEGREES, not RADIANS
        self.front_tire_coeff_Fy = [1.69, 0.0004384, 2.769, 614.3, 1496, 0.01784, 0.000432, 0.7237, 0.0001746, 0.1366,
                        -0.1482, -0.06455, 10.45, 3.036e-05, 0.04111, 0.002054, 0.01834, -0.06673]
        self.rear_tire_coeff_Fy = [1.384, -0.0003117, -2.936, 668.1, 1599, 0.03877, 0.0003177, 0.6252, 7.733e-05, -0.08382,
                -0.1171, 0.04597, 3.107, 5.41e-05, 0.04736, 0.005249, 0.0508, -0.1956]
        
        self.front_tire_spring_coeffs = [624 * 175, 0.5 / 0.0254] # N/m
        self.rear_tire_spring_coeffs = [715.3 * 175, 0.486 / 0.0254] # N/m
        
        # TODO: implement aligning moment & fitting
        #self.front_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        #self.rear_tire_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        
        #TODO: implement accel/braking corner & fitting
        #self.rear_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        #self.front_tire_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        
        
        
        ### aerodynamics params ###
        
        self.air_temperature = 27 # Celsius
        self.ClA_tot = 2.93
        self.CdA_tot = 1.024
        self.CsA_tot = 33.91
        self.CdA0 = 0.7155 # drag coefficient from non aero componenets
        self.ride_height = 0.0762 # m # TODO: investigate if this is correct

    @property
    def cg_weighted_track(self): # m
        return (self.front_track * (1 - self.cg_bias) + self.rear_track * self.cg_bias) / 2

    @property
    def cg_total_position(self): # m
        return np.array([self.cg_bias * self.wheelbase, (self.cg_left - 0.5) * self.cg_weighted_track, self.cg_height])
    
    @property
    def mass(self): # kg
        return self.mass_sprung + 2 * self.mass_unsprung_front + 2 * self.mass_unsprung_rear