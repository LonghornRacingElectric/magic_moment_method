from abc import abstractmethod, abstractproperty
import math
import numpy as np
from scipy.optimize import fmin

class Tire:
    def __init__(self, car_params, direction_left):
        self.params = car_params
        self.direction_left = direction_left # boolean

    def roll_inclination_angle_gain(self, roll):
        mod_roll = - roll if self.direction_left else roll
        return mod_roll - (mod_roll * self.camber_gain)
    
    @abstractmethod
    def steered_inclination_angle_gain(self):
        pass

    def is_saturated(self, normal_force:float, slip_angle:float, inclination_angle:float):
        guess_peak_slip_angle = 18 * np.pi / 180 # rad
        # vvv would work if pacejka fits were any good lol
        #max_slip_angle = fmin(lambda x: -self.lateral_pacejka(inclination_angle, normal_force, x), guess_peak_slip_angle)
        return slip_angle > guess_peak_slip_angle
        
    def roll_force(self, roll:float):
        # TODO: do about roll center
        return self.roll_stiffness * roll

    def heave_force(self, heave:float):
        return self.riderate * heave

    def pitch_force(self, pitch:float):
        # TODO: implement antidive & antipitch
        # TODO: do about pitch center
        effective_heave = self.position[0] * np.sin(pitch)
        return self.riderate * effective_heave
          
    # TODO: re-implement loss function
    # calculate inclination_angle losses!
    # lateral_force_no_IA = self.lateral_pacejka(0, normal_load, slip_degrees) * multiplier
    # self.outputs.inclination_angle_percent_loss = (lateral_force_no_IA - lateral_force) / lateral_force_no_IA
    # self.outputs.inclination_angle_force_loss = (lateral_force_no_IA - lateral_force)

    # Determines the lateral force on the tire given the pacejka fit coefficients, slip angle, camber, and normal load
    # https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
    # NOTE - ATM inclination angle and slip angle will be converted to ~~DEGREES~~ inside here
    def lateral_pacejka(self, inclination_angle, normal_force, slip_angle):
        # NOTE: 1/-1 multiplier on slip_degrees is done for any non-symmetries in fit
        multiplier =  -1 if self.direction_left else 1
        slip_degrees = slip_angle * 180 / math.pi * multiplier # degrees
        inclination_degrees = inclination_angle * 180 / math.pi # degrees
        
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17] = self.lateral_coeffs
        C = a0
        D = normal_force * (a1 * normal_force + a2) * (1 - a15 * inclination_degrees ** 2)
        BCD = a3 * math.sin(math.atan(normal_force / a4) * 2) * (1 - a5 * abs(inclination_degrees))
        B = BCD / (C * D)
        H = a8 * normal_force + a9 + a10 * inclination_degrees
        E = (a6 * normal_force + a7) * (1 - (a16 * inclination_degrees + a17) * math.copysign(1, slip_degrees + H))
        V = a11 * normal_force + a12 + (a13 * normal_force + a14) * inclination_degrees * normal_force
        Bx1 = B * (slip_degrees + H)
        
        # NOTE: 2/3 multiplier comes from TTC forum suggestions, including from Bill Cobb
        test_condition_multiplier = 2/3
        return test_condition_multiplier * (D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V) * multiplier

    # def long_pacejka(self, inclination_angle, normal_force, slip_ratio):
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

    # sees how much force is being lost if inclination angle was optimal (0 based on initial TTC data)
    def lateral_loss(self, normal_force, slip_angle, inclination_angle):
        if normal_force <= 0:
            return 0, 0
        actual = self.lateral_pacejka(inclination_angle, normal_force, slip_angle)
        optimal = self.lateral_pacejka(0, normal_force, slip_angle)
        force_loss = optimal - actual
        percent_loss = force_loss / optimal
        return force_loss, percent_loss

    # TODO: this may not be entirely correct right now
    @property
    def roll_stiffness(self): # Nm/rad (assumes small angle approximation)
        ride_contribution = self.riderate * self.trackwidth ** 2 / 2 * (1 if self.direction_left else -1)
        arb_contribution = self.arb_stiffness
        return (ride_contribution + arb_contribution)

    @abstractmethod
    def steering_induced_slip(self, steered_angle):
        pass

    @abstractmethod
    def steered_inclination_angle_gain(self, steered_angle):
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