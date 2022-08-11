from abc import abstractproperty
from helpers.better_namespace import BetterNamespace
import numpy as np

# ALL STATIC PARAMETERS GO HERE (or parameters assumed to be static)
class BaseVehicle(BetterNamespace):
    def __init__(self):
        super().__init__()

    ### ~~~ VEHICLE PARAMETERS ~~~ ###

    @abstractproperty
    def sprung_inertia(self):
        pass

    @abstractproperty
    def gravity(self):
        pass

    @abstractproperty
    def cg_bias(self):
        pass

    @abstractproperty
    def cg_left(self):
        pass

    @abstractproperty
    def cg_height(self):
        pass

    @abstractproperty
    def wheelbase(self):
        pass

    @abstractproperty
    def front_track(self):
        pass

    @abstractproperty
    def rear_track(self):
        pass

    @abstractproperty
    def mass_unsprung_front(self):
        pass

    @abstractproperty
    def mass_unsprung_rear(self):
        pass

    @abstractproperty
    def driver_mass(self):
        pass

    @abstractproperty
    def mass_sprung(self):
        pass

    ### ~~~ SUSPENSION PARAMETERS ~~~ ###

    @abstractproperty
    def front_spring_springrate(self):
        pass

    @abstractproperty
    def rear_spring_springrate(self):
        pass

    @abstractproperty
    def front_motion_ratio(self):
        pass

    @abstractproperty
    def rear_motion_ratio(self):
        pass

    @abstractproperty
    def antidive(self):
        pass

    @abstractproperty
    def front_arb_stiffness(self):
        pass

    @abstractproperty
    def rear_arb_stiffness(self):
        pass

    @abstractproperty
    def rear_toe(self):
        pass

    @abstractproperty
    def front_toe(self):
        pass

    @abstractproperty
    def front_static_camber(self):
        pass

    @abstractproperty
    def rear_static_camber(self):
        pass

    @abstractproperty
    def rear_camber_gain(self):
        pass

    @abstractproperty
    def front_camber_gain(self):
        pass

    @abstractproperty
    def front_KPI(self):
        pass

    @abstractproperty
    def rear_KPI(self):
        pass

    @abstractproperty
    def front_caster(self):
        pass

    @abstractproperty
    def rear_caster(self):
        pass

    @abstractproperty
    def rear_toe_gain(self):
        pass

    @abstractproperty
    def front_roll_center_height(self):
        pass

    @abstractproperty
    def rear_roll_center_height(self):
        pass

    ### ~~~ TIRES & PACEJKA ~~~ ###
    @abstractproperty
    def front_tire_coeff_Fy(self):
        pass

    @abstractproperty
    def rear_tire_coeff_Fy(self):
        pass

    @abstractproperty
    def front_tire_spring_coeffs(self):
        pass

    @abstractproperty
    def rear_tire_spring_coeffs(self):
        pass
        
    ### ~~~ AERODYNAMIC PARAMETERS ~~~ ###
    @abstractproperty
    def air_temperature(self):
        pass

    @abstractproperty
    def ClA_tot(self):
        pass

    @abstractproperty
    def CdA_tot(self):
        pass

    @abstractproperty
    def CsA_tot(self):
        pass

    @abstractproperty
    def CdA0(self):
        pass

    @abstractproperty
    def static_ride_height(self):
        pass

    ### ~~~ CALCULATED PARAMETERS ~~~ ###

    @property
    def cg_weighted_track(self): # m
        return (self.front_track * (1 - self.cg_bias) + self.rear_track * self.cg_bias) / 2

    @property
    def cg_total_position(self): # m
        return np.array([self.cg_bias * self.wheelbase, (self.cg_left - 0.5) * self.cg_weighted_track, self.cg_height])
    
    @property
    def mass(self): # kg
        return self.mass_sprung + 2 * self.mass_unsprung_front + 2 * self.mass_unsprung_rear