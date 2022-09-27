from abc import abstractmethod, abstractproperty
import math
import numpy as np
from scipy.optimize import fmin
import warnings

class Tire:
    def __init__(self, car_params, is_left_tire):
        self.params = car_params
        self.direction_left = is_left_tire # boolean

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
          
    # TODO: re-implement loss function
    # calculate inclination_angle losses!
    # lateral_force_no_IA = self.lateral_pacejka(0, normal_load, slip_degrees) * multiplier
    # self.outputs.inclination_angle_percent_loss = (lateral_force_no_IA - lateral_force) / lateral_force_no_IA
    # self.outputs.inclination_angle_force_loss = (lateral_force_no_IA - lateral_force)

    # Determines the lateral force on the tire given the pacejka fit coefficients, slip angle, camber, and normal load
    # https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
    # NOTE - ATM inclination angle and slip angle will be converted to ~~DEGREES~~ inside here
    def lateral_pacejka(self, inclination_angle:float, normal_force:float, slip_angle:float):
        if normal_force <= 0:
            return 0
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
    
    warnings.filterwarnings("error")
    def longitudinal_pacejka(self, normal_force:float, slip_ratio:float):
        FZ = normal_force / 1000
        SR = slip_ratio
        
        try:
            [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13] = self.longitudinal_coeffs
            C = b0
            D = FZ * (b1 * FZ + b2)

            BCD = (b3 * FZ**2 + b4 * FZ) * np.exp(-1 * b5 * FZ)

            B = BCD / (C * D)

            H = b9 * FZ + b10

            E = (b6 * FZ**2 + b7 * FZ + b8) * (1 - b13 * np.sign(SR + H))

            V = b11 * FZ + b12
            Bx1 = B * (SR + H)

            return (D * np.sin(C * np.arctan(Bx1 - E * (Bx1 - np.arctan(Bx1)))) + V)
        
        except RuntimeWarning:
            return 1
        
    # Long and lat formulas from Comstock
    def com_lat(self, SA, SR, FX, FY, Cs):
        SR_adj = SR / 100
        SA_adj = SA * np.pi / 180
        return ((FX * FY) / np.sqrt(SR_adj**2 * FY**2 + FX**2 * (np.tan(SA_adj))**2)) * (np.sqrt((1 - SR_adj)**2 * (np.cos(SA_adj))**2 * FY**2 + (np.sin(SA_adj))**2 * Cs**2) / (Cs * np.cos(SA_adj)))

    def com_long(self, SA, SR, FX, FY, Ca):
        SR_adj = SR / 100
        SA_adj = SA * np.pi / 180
        return ((FX * FY) / np.sqrt(SR_adj**2 * FY**2 + FX**2 * (np.tan(SA_adj))**2)) * (np.sqrt(SR_adj**2 * Ca**2 + (1 - SR_adj)**2 * (np.cos(SA_adj))**2 * FX**2) / Ca)
    
    # Full comstock calculations
    def comstock_lat(self, SR, SA, FZ, IA):
        SR *= 100
        FX = self.longitudinal_pacejka(FZ, SR)
        FY = self.lateral_pacejka(FZ, SA, IA)

        Ca = (self.longitudinal_pacejka(FZ, 1) - self.longitudinal_pacejka(FZ, 0)) * (180 / np.pi)
        Cs = (self.lateral_pacejka(FZ, 1, 0) - self.lateral_pacejka(FZ, 0, 0)) * 100
        
        if abs(SR) < 5 and abs(SA) < 5:
            return FY
        elif abs(SA) < 5:
            return FY
        elif abs(SR) < 5:
            return self.com_lat(SA, SR, FX, FY, Cs)
        else:
            return self.com_lat(SA, SR, FX, FY, Cs)
    
    def comstock_long(self, SR, SA, FZ, IA):
        FX = self.longitudinal_pacejka(FZ, SR)
        FY = self.lateral_pacejka(FZ, SA, IA)

        Ca = (self.longitudinal_pacejka(FZ, 1) - self.longitudinal_pacejka(FZ, 0)) * (180 / np.pi)
        Cs = (self.lateral_pacejka(FZ, 1, 0) - self.lateral_pacejka(FZ, 0, 0)) * 100
        
        if abs(SR) < 1 and abs(SA) < 1:
            return FX
        elif abs(SA) < 1:
            return self.com_long(SA, SR, FX, FY, Ca)
        elif abs(SR) < 1:
            return FX
        else:
            return self.com_long(SA, SR, FX, FY, Ca)

    # sees how much force is being lost if inclination angle was optimal (0 based on initial TTC data)
    def lateral_loss(self, normal_force:float, slip_angle:float, inclination_angle:float):
        actual = self.lateral_pacejka(inclination_angle, normal_force, slip_angle)
        optimal = self.lateral_pacejka(0, normal_force, slip_angle)
        force_loss = optimal - actual
        percent_loss = force_loss / optimal if optimal > 0 else 0
        return force_loss, percent_loss

    def tire_stiffness_func(self, tire_displacement:float):
        # K = C0 + C1 * Nf
        # Nf = K * x
        return self.tire_coeffs[0] / (1 - self.tire_coeffs[1] * tire_displacement)

    def wheelrate_f(self, spring_displacement:float):
        return self.wheelrate

    @abstractmethod
    def steering_induced_slip(self, steered_angle:float):
        pass

    @abstractmethod
    def steered_inclination_angle_gain(self, steered_angle:float):
        pass

    @abstractproperty
    def wheelrate(self):
        pass

    @abstractproperty
    def toe(self):
        pass
    
    @abstractproperty
    def toe_gain(self):
        pass
    
    @abstractproperty
    def tire_coeffs(self):
        pass

    @abstractproperty
    def lateral_coeffs(self):
        pass
    
    @abstractproperty
    def longitudinal_coeffs(self):
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

    @abstractproperty
    def radius(self):
        pass

    @abstractmethod
    def get_slip_ratio(self, slip_ratios):
        pass