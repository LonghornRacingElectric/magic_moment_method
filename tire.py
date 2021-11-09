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
    def __init__(self, car_params, location, direction_left):
        self.params = car_params
        self.direction_left = direction_left # Boolean
        self.position = np.array(location)  # Position of the center of the contact patch relative to vehicle frame

        self.outputs = BetterNamespace()
        self.outputs.unsprung_displacement = None
        self.outputs.tire_centric_forces = None # tire forces in the tire coordinate system
        self.outputs.velocity = None
        self.outputs.slip_angle = None
        self.outputs.inclination_angle = 0
        self.outputs.vehicle_centric_forces = None # tire forces in the vehicle coordinate system
        self.outputs.moments = None
        self.outputs.z_c = None
        self.outputs.f_roll = None
        self.outputs.f_heave = None
        #self.outputs.slip_ratio = None

    @property
    def normal_load(self):
        return self.stiffness * self.outputs.unsprung_displacement

    @property
    def static_unsprung_displacement(self):
        return self.static_normal_load / self.stiffness
    
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
        Fz = self.normal_load
        if Fz < 0:
            self.outputs.vehicle_centric_forces = np.array([0, 0, 0])
            self.outputs.tire_centric_forces = np.array([0, 0, 0])
            self.outputs.moments = np.zeros(3)
        else:
            # TODO: need to consider slip ratio
            # multiplier is done for any non-symmetries in fit
            multiplier = -1 if self.direction_left else 1
            slip_degrees = self.outputs.slip_angle * 180/math.pi * multiplier

            [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16,
                a17] = self.lateral_coeffs
            
            inclination_angle = self.outputs.inclination_angle

            C = a0
            D = Fz * (a1 * Fz + a2) * (1 - a15 * inclination_angle ** 2)
            BCD = a3 * math.sin(math.atan(Fz / a4) * 2) * (1 - a5 * abs(inclination_angle))
            B = BCD / (C * D)
            H = a8 * Fz + a9 + a10 * inclination_angle
            E = (a6 * Fz + a7) * (1 - (a16 * inclination_angle + a17) * math.copysign(1, slip_degrees + H))
            V = a11 * Fz + a12 + (a13 * Fz + a14) * inclination_angle * Fz
            Bx1 = B * (slip_degrees + H)

            Fy = multiplier * 2/3* (D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V)

            self.outputs.tire_centric_forces = np.array([0, Fy, self.normal_load])

            # Rotate the tire force into intermediate frame and add moments/forces to total
            # TODO: conversions utilities
            slip_radians = self.outputs.slip_angle
            rotation_matrix = np.array([[cos(slip_radians),-sin(slip_radians),0],[sin(slip_radians),cos(slip_radians),0],[0,0,1]])
            self.outputs.vehicle_centric_forces = rotation_matrix.dot(self.outputs.tire_centric_forces)

            self.outputs.moments = np.cross(self.outputs.vehicle_centric_forces, self.position)

        return self.outputs.vehicle_centric_forces, self.outputs.moments 

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
    def __init__(self, car_params, location, direction_left):
        super().__init__(car_params, location, direction_left)

    def steering_induced_slip(self, steered_angle):
        return steered_angle + self.toe

    @property
    def toe(self):
        return self.params.front_toe * (1 if self.direction_left else -1)

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

class RearTire(Tire):
    def __init__(self, car_params, location, direction_left):
        super().__init__(car_params, location, direction_left)

    # NOT a function of steered angle, don't use for calcs
    def steering_induced_slip(self, steered_angle):
        return self.toe

    @property
    def toe(self):
        return self.params.rear_toe * (1 if self.direction_left else -1)

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