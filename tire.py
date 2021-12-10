from abc import abstractmethod, abstractproperty
from math import sin, cos
import math
import numpy as np
from better_namespace import BetterNamespace

"""
Coordinate Systems:
    SAE z-up wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html

Units: 
    Length: meters
    Mass: kg
"""
class Tire:
    def __init__(self, car_params, direction_left):
        self.params = car_params
        self.direction_left = direction_left # boolean
        
        self.outputs = BetterNamespace()
        self.outputs.unsprung_displacement = None # m
        self.outputs.tire_centric_forces = None # N; tire forces in the tire coordinate system
        self.outputs.velocity = None # m/s
        self.outputs.slip_angle = None # rad
        self.outputs.inclination_angle = None # rad
        self.outputs.vehicle_centric_forces = None # N; tire forces in the vehicle coordinate system
        self.outputs.moments = None # N*m
        self.outputs.steering_inclination = None # rad
        self.outputs.z_c = None # m
        self.outputs.f_roll = None # N
        self.outputs.f_heave = None # N
        self.outputs.inclination_angle_percent_loss = None # %; loss in lateral force production due to IA
        self.outputs.inclination_angle_force_loss = None # N

        #Tube Forces
        self.outputs.tube_force_FUCA = None #N
        self.outputs.tube_force_RUCA = None #N
        self.outputs.tube_force_FLCA = None # N
        self.outputs.tube_force_RLCA = None # N
        self.outputs.tube_force_p_rod = None #N
        self.outputs.tube_force_toe_link = None #N
        #self.outputs.slip_ratio = None # not implemented

    @property
    def lifting(self):
        return self.outputs.tire_centric_forces[2] == 0

    @property
    def wheel_displacement(self):
        return self.outputs.z_c - self.params.ride_height + self.outputs.unsprung_displacement

    @property
    def normal_load(self):
        return self.stiffness * self.outputs.unsprung_displacement

    @property
    def static_unsprung_displacement(self):
        return self.static_normal_load / self.stiffness
    
    # input steered angle is in intermediate frame
    # TODO: should toe be included in this steered angle or added afterwards?
    def steered_inclination_angle_gain(self, steered_angle):
        # convert steered angle to tire frame
        steered_angle = steered_angle * (1 if self.direction_left else -1) + self.toe
        
        # steer_inc = - tire.caster * delta + (1 / 2) * tire.KPI * np.sign(delta) * (delta ** 2)
        steer_inc = np.arccos(np.sin(self.KPI) * np.cos(steered_angle)) + self.KPI + \
                    np.arccos(np.sin(self.caster) * np.sin(steered_angle)) - math.pi
                    
        return steer_inc

    @property
    def is_saturated(self):
        # TODO: implement better method
        peak_slip_angle = 18 * math.pi / 180 # rad
        return self.outputs.slip_angle > peak_slip_angle
        
    # takes corner displacement of chassis and roll angle
    def set_unsprung_displacement(self, z_c, roll):
        self.outputs.z_c = z_c
        self.outputs.f_roll = self.roll_stiffness * roll # ARB force at tire

        unsprung_disp = (self.outputs.f_roll + self.wheelrate * z_c) / (self.stiffness + self.wheelrate)
        
        self.outputs.f_heave = self.wheelrate * (z_c - unsprung_disp) # spring force at tire
        self.outputs.unsprung_displacement = unsprung_disp

    # Determines the lateral force on the tire given the pacejka fit coefficients, slip angle, camber, and normal load
    # https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
    def get_loads(self):
        # note: don't allow normal force of 0 to produce tire forces
        if self.normal_load < 0:
            self.outputs.vehicle_centric_forces = np.array([0, 0, 0])
            self.outputs.tire_centric_forces = np.array([0, 0, 0])
            self.outputs.moments = np.zeros(3)
            self.outputs.inclination_angle_percent_loss = 0
            self.outputs.inclination_angle_force_loss = 0
            self.outputs.tube_force_FUCA = 0  # N
            self.outputs.tube_force_RUCA = 0  # N
            self.outputs.tube_force_FLCA = 0  # N
            self.outputs.tube_force_RLCA = 0  # N
            self.outputs.tube_force_p_rod = 0  # N
            self.outputs.tube_force_toe_link = 0  # N
        else:
            # TODO: need to consider slip ratio
            # NOTE: 1/-1 multiplier on slip_degrees is done for any non-symmetries in fit
            multiplier =  -1 if self.direction_left else 1
            slip_degrees = self.outputs.slip_angle * 180 / math.pi * multiplier # degrees
            inclination_angle = self.outputs.inclination_angle * 180 / math.pi # degrees

            lateral_force = self.lateral_pacejka(inclination_angle, self.normal_load, slip_degrees) * multiplier

            self.outputs.tire_centric_forces = np.array([0, lateral_force, self.normal_load])

            # Rotate the tire force into intermediate frame
            self.outputs.vehicle_centric_forces = self.tire_heading_frame_to_intermediate_transform(self.outputs.tire_centric_forces)
            self.outputs.moments = np.cross(self.outputs.vehicle_centric_forces, self.position)

            # calculate inclination_angle losses!
            lateral_force_no_IA = self.lateral_pacejka(0, self.normal_load, slip_degrees) * multiplier
            self.outputs.inclination_angle_percent_loss = (lateral_force_no_IA - lateral_force) / lateral_force_no_IA
            self.outputs.inclination_angle_force_loss = (lateral_force_no_IA - lateral_force)

            # calculate tube forces
            self.outputs.tube_force_FUCA, self.outputs.tube_force_RUCA, self.outputs.tube_force_FLCA \
            , self.outputs.tube_force_RLCA, self.outputs.tube_force_p_rod, self.outputs.tube_force_toe_link \
                    = [float(x) for x in self.get_tube_forces(self.outputs.vehicle_centric_forces * np.array([1, multiplier, 1]))]

        return self.outputs.vehicle_centric_forces, self.outputs.moments 

    # Transform input array to Intermediate Frame from tire heading using tire slip angle
    def tire_heading_frame_to_intermediate_transform(self, tire_heading_frame_vector):
        rotation_matrix = np.array([[cos(self.outputs.slip_angle), -sin(self.outputs.slip_angle), 0],
                                    [sin(self.outputs.slip_angle), cos(self.outputs.slip_angle),0],
                                    [0,0,1]])
        return np.dot(rotation_matrix, tire_heading_frame_vector)     

    # TODO; BIG NOTE - ATM inclination angle and slip angle MUST be taken in ~~DEGREES~~ here
    def lateral_pacejka(self, inclination_angle, normal_force, slip_degrees):
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17] = self.lateral_coeffs
        C = a0
        D = normal_force * (a1 * normal_force + a2) * (1 - a15 * inclination_angle ** 2)
        BCD = a3 * math.sin(math.atan(normal_force / a4) * 2) * (1 - a5 * abs(inclination_angle))
        B = BCD / (C * D)
        H = a8 * normal_force + a9 + a10 * inclination_angle
        E = (a6 * normal_force + a7) * (1 - (a16 * inclination_angle + a17) * math.copysign(1, slip_degrees + H))
        V = a11 * normal_force + a12 + (a13 * normal_force + a14) * inclination_angle * normal_force
        Bx1 = B * (slip_degrees + H)
        
        # NOTE: 2/3 multiplier comes from TTC forum suggestions, including from Bill Cobb
        test_condition_multiplier = 2/3
        return test_condition_multiplier * (D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V)

    def get_tube_forces(self, tire_forces):
        normal_vects = self.tube_geometry[0]
        lever_arms = self.tube_geometry[1]

        # Set Up Force and Moment Balance
        arr_cross = np.cross(normal_vects.T, lever_arms.T).T
        arr_coeff = np.concatenate((normal_vects, arr_cross))

        # Solve
        b = np.concatenate((tire_forces, np.zeros(3)))
        #tube_forces = np.expand_dims(np.linalg.solve(arr_coeff, b), axis=0)
        tube_forces = np.linalg.solve(arr_coeff, b)

        return tube_forces

    @abstractmethod
    def steering_induced_slip(self, steered_angle):
        pass

    @abstractproperty
    def toe(self):
        pass
    
    @abstractproperty
    def stiffness(self):
        pass

    @abstractproperty
    def lateral_coeffs(self):
        pass

    @abstractproperty
    def static_normal_load(self):
        pass

    @abstractproperty
    def roll_stiffness(self):
        pass

    @abstractproperty
    def wheelrate(self):
        pass

    @abstractproperty
    def KPI(self):
        pass

    @abstractproperty
    def static_camber(self):
        pass

    @abstractproperty
    def camber_gain(self):
        pass

    @abstractproperty
    def caster(self):
        pass
    
    @abstractproperty
    def trackwidth(self):
        pass

    @abstractproperty
    def position(self):
        pass

    @abstractproperty
    def tube_geometry(self):
        pass

    # def get_Fx(self, slip_angle, camber, Fz):
    #     # [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15, b16, b17] = self.pacejka_fit.Fx_coefficients
    #     # C = b0;
    #     # D = Fz*(b1*Fz+b2)
    #     # BCD = (b3**Fz+b4*Fz)*math.exp(-b5*Fz)
    #     # B = BCD/(C*D)
    #     # H = b9*Fz+b10
    #     # E = (b6**Fz+b7*Fz+b8)*(1-b13*math.sign(slip_ratio+H))
    #     # V = b11*Fz+b12
    #     # Bx1 = B*(slip_ratio+H)

    #     # return D*math.sin(C*math.atan(Bx1-E*(Bx1-math.atan(Bx1)))) + V
    #     return 0

class FrontTire(Tire):
    def __init__(self, car_params, direction_left):
        super().__init__(car_params, direction_left)

    def steering_induced_slip(self, steered_angle):
        return steered_angle + self.toe

    @property
    def toe(self):
        return self.params.front_toe * (1 if self.direction_left else -1) # TODO: verify this

    @property
    def stiffness(self):
        return self.params.front_tire_spring_rate
    
    @property
    def static_normal_load(self):
        return -1 * self.params.gravity * self.params.mass * (1 - self.params.cg_bias) / 2

    @property
    def lateral_coeffs(self):
        return self.params.front_tire_coeff_Fy

    @property
    def roll_stiffness(self):
        return self.params.front_roll_stiffness * (1 if self.direction_left else -1)

    @property
    def wheelrate(self):
        return self.params.front_wheelrate_stiffness

    @property
    def KPI(self):
        return self.params.front_KPI

    @property
    def static_camber(self):
        return self.params.front_static_camber

    @property
    def camber_gain(self):
        return self.params.front_camber_gain

    @property
    def caster(self):
        return self.params.front_caster

    @property
    def trackwidth(self):
        return self.params.front_track

    @property
    def position(self):
        y_pos = self.trackwidth/2 * (1 if self.direction_left else -1)
        return [self.params.wheelbase * self.params.cg_bias, y_pos, 0]

    @property
    def tube_geometry(self):
        # Inputs (in inches, from CAD)
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

        # Calculate
        pt_i_arr = np.concatenate((pt1_i, pt2_i, pt3_i, pt4_i, pt5_i, pt6_i), axis=1)
        pt_f_arr = np.concatenate((pt1_f, pt2_f, pt3_f, pt4_f, pt5_f, pt6_f), axis=1)

        v_arr = pt_f_arr - pt_i_arr
        lengths = np.apply_along_axis(np.linalg.norm, 0, v_arr)
        n_arr = v_arr / lengths

        lever_arms = pt_i_arr * 0.0254  # in to m

        return n_arr, lever_arms


class RearTire(Tire):
    def __init__(self, car_params, direction_left):
        super().__init__(car_params, direction_left)

    # NOT a function of steered angle, don't use for calcs
    def steering_induced_slip(self, steered_angle):
        return self.toe

    @property
    def toe(self):
        return self.params.rear_toe * (1 if self.direction_left else -1) #TODO : verify this

    # TODO: make linear instead of constant
    @property
    def stiffness(self):
        return self.params.rear_tire_spring_rate
    
    @property
    def static_normal_load(self):
        return -1 * self.params.gravity * self.params.mass * self.params.cg_bias / 2

    @property
    def lateral_coeffs(self):
        return self.params.rear_tire_coeff_Fy

    @property
    def roll_stiffness(self):
        return self.params.rear_roll_stiffness * (1 if self.direction_left else -1)

    @property
    def wheelrate(self):
        return self.params.rear_wheelrate_stiffness

    @property
    def KPI(self):
        return self.params.rear_KPI

    @property
    def static_camber(self):
        return self.params.rear_static_camber

    @property
    def camber_gain(self):
        return self.params.rear_camber_gain

    @property
    def caster(self):
        return self.params.rear_caster
    
    @property
    def trackwidth(self):
        return self.params.rear_track

    @property
    def position(self):
        y_pos = self.trackwidth/2 * (1 if self.direction_left else -1)
        return [-self.params.wheelbase * (1 - self.params.cg_bias), y_pos, 0]

    @property
    def tube_geometry(self):
        # Inputs (in inches, from CAD)
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

        # Calculate
        pt_i_arr = np.concatenate((pt1_i, pt2_i, pt3_i, pt4_i, pt5_i, pt6_i), axis=1)
        pt_f_arr = np.concatenate((pt1_f, pt2_f, pt3_f, pt4_f, pt5_f, pt6_f), axis=1)

        v_arr = pt_f_arr - pt_i_arr
        lengths = np.apply_along_axis(np.linalg.norm, 0, v_arr)
        n_arr = v_arr / lengths

        lever_arms = pt_i_arr * 0.0254  # in to m

        return n_arr, lever_arms
