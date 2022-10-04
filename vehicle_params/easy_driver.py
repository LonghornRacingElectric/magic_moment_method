import math
import numpy as np

# ALL STATIC PARAMETERS GO HERE (or parameters assumed to be static)
class EasyDriver:
    def __init__(self):
        #super().__init__()

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
        
        # ~~~ Tires & Pacejka ~~~ #
        # NOTE: These lateral fits all assumed slip angle was in DEGREES, not RADIANS
        self.front_tire_coeff_Fy = [0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0.000158, 0.149, 
                        -0.1595, 0.0329, 9.153,  0.00001406, 0.0328, 0.00362, -0.0143, -0.0116]

        self.front_tire_coeff_Fx = [0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175, -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0.3977386758725586, 0, 0, 0, 0.10106424367287903]

        self.rear_tire_coeff_Fy = [1.384, -0.0003117, -2.936, 668.1, 1599, 0.03877, 0.0003177, 0.6252, 7.733e-05, -0.08382, -0.1171, 0.04597, 
                        3.107, 5.41e-05, 0.04736, 0.005249, 0.0508, -0.1956]

        self.rear_tire_coeff_Fx = [0.46024966176377113, 4000.509873697152, 1097.1712081460967, 202.18848632159495, 100.8812198037175, -0.2557010431649166, 0.3066955241461764, 0.011822770671297778, -1.9521015799737094, 0.3977386758725586, 0, 0, 0, 0.10106424367287903]
        
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
        self.ClA_tot = 4.384
        self.CdA_tot = 1.028
        self.CsA_tot = 33.91
        self.CdA0 = 0.7155 # drag coefficient from non aero componenets
        self.static_ride_height = 0.0762 # m



        ### differential & braking params ###

        self.motor_radius = 1
        self.diff_radius = 4
        self.front_tire_radius = 8 * .0254
        self.rear_tire_radius = 9 * .0254
        self.motor_inertia = 0.1
        self.diff_inertia = 0.1
        self.motor_damping = 0.1
        self.diff_damping = 0.1
        self.driveline_inertias = np.array([0.1, 0.1, 0.1, 0.1])
        self.driveline_damping = np.array([0.1, 0.1, 0.1, 0.1])
        self.diff_efficiency = 1
        self.max_pedal_force = 150
        self.pedal_ratio = 3
        self.master_cylinder_area = 0.2
        self.brake_bias_ratio = 0.6 # Percent of front
        self.rotor_radius = [0.3 * 7, 0.2 * 9]
        self.calipers_area = [0.2, 0.2]
        self.brake_pad_mu = [0.55, 0.55]
        self.diff_fl = 0.607
        self.diff_preload = 5.2
        self.max_torque = 230 # Nm


    @property
    def cg_weighted_track(self): # m
        return (self.front_track * (1 - self.cg_bias) + self.rear_track * self.cg_bias) / 2

    @property
    def cg_total_position(self): # m
        return np.array([self.cg_bias * self.wheelbase, (self.cg_left - 0.5) * self.cg_weighted_track, self.cg_height])
    
    @property
    def mass(self): # kg
        return self.mass_sprung + 2 * self.mass_unsprung_front + 2 * self.mass_unsprung_rear