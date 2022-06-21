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
        #self.outputs.slip_ratio = None # not implemented

    @property
    def lifting(self):
        return self.outputs.tire_centric_forces[2] == 0

    @property
    def wheel_displacement(self):
        return self.outputs.z_c - self.params.ride_height + self.outputs.unsprung_displacement

    @property
    def normal_load(self):
        return self.tire_springrate * self.outputs.unsprung_displacement

    @property
    def static_unsprung_displacement(self):
        return self.static_normal_load / self.tire_springrate
    
    @abstractmethod
    def steered_inclination_angle_gain(self):
        pass

    @property
    def is_saturated(self):
        # TODO: implement better method
        peak_slip_angle = 18 * math.pi / 180 # rad
        return self.outputs.slip_angle > peak_slip_angle
        
    # takes corner displacement of chassis and roll angle
    def set_unsprung_displacement(self, z_c, roll):
        self.outputs.z_c = z_c
        self.outputs.f_roll = self.arb_stiffness * roll # TODO: more long term fix

        unsprung_disp = (self.outputs.f_roll + self.wheelrate * z_c) / (self.tire_springrate + self.wheelrate)
        
        self.outputs.f_heave = self.riderate * (z_c )# self.params.ride_height) # spring force at tire
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

    @property
    def roll_stiffness(self): # Nm/rad (assumes small angle approximation)
        ride_contribution = self.riderate * self.trackwidth ** 2 / 2 * (1 if self.direction_left else -1)
        arb_contribution = self.arb_stiffness
        return (ride_contribution + arb_contribution)

    @abstractmethod
    def steering_induced_slip(self, steered_angle):
        pass

    @abstractproperty
    def riderate(self):
        pass
    
    @abstractproperty
    def arb_stiffness(self):
        pass

    @abstractproperty
    def toe(self):
        pass
    
    @abstractproperty
    def toe_gain(self):
        pass
    
    @abstractproperty
    def tire_springrate(self):
        pass

    @abstractproperty
    def lateral_coeffs(self):
        pass

    @abstractproperty
    def static_normal_load(self):
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