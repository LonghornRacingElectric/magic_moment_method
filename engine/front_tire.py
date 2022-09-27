
from engine.tire import Tire
import numpy as np

class FrontTire(Tire):
    def __init__(self, car_params, is_left_tire):
        super().__init__(car_params, is_left_tire)

    def steering_induced_slip(self, steered_angle): # rad
        return steered_angle + self.toe

    @property
    def toe(self): # rad
        return self.params.front_toe * (1 if self.direction_left else -1) # TODO: verify this

    @property
    def tire_coeffs(self):
        return self.params.front_tire_spring_coeffs
    
    @property
    def lateral_coeffs(self):
        return self.params.front_tire_coeff_Fy
    
    @property
    def longitudinal_coeffs(self):
        return self.params.front_tire_coeff_Fx

    @property
    def arb_stiffness(self): # Nm / rad
        return self.params.front_arb_stiffness * (1 if self.direction_left else -1)

    # NOTE: Explanation of motion ratios & wheelrates: https://en.wikipedia.org/wiki/Motion_ratio
    @property
    def wheelrate(self): # N/m
        return self.params.front_spring_springrate/self.params.front_motion_ratio**2

    @property
    def KPI(self): # rad
        return self.params.front_KPI

    @property
    def static_camber(self): # rad
        return self.params.front_static_camber

    @property
    def camber_gain(self): # rad/rad
        return self.params.front_camber_gain

    @property
    def caster(self): # rad
        return self.params.front_caster

    @property
    def trackwidth(self): # m
        return self.params.front_track

    @property
    def position(self): # [m, m, m]
        y_pos = self.trackwidth/2 * (1 if self.direction_left else -1)
        return [self.params.wheelbase * self.params.cg_bias, y_pos, 0]
    
    # input steered angle is in intermediate frame
    def steered_inclination_angle_gain(self, steered_angle):
        # convert steered angle to tire frame
        steered_angle = steered_angle * (1 if self.direction_left else -1)
        
        # steer_inc = - tire.caster * delta + (1 / 2) * tire.KPI * np.sign(delta) * (delta ** 2)
        steer_inc = np.arccos(np.sin(self.KPI) * np.cos(steered_angle)) + self.KPI + \
                    np.arccos(np.sin(self.caster) * np.sin(steered_angle)) - np.pi
                    
        return steer_inc

    @property
    def tube_geometry(self):
        return self.params.front_tube_normals, self.params.front_lever_arms

    @property
    def radius(self):
        return self.params.front_tire_radius

    def get_slip_ratio(self, slip_ratios):
        return slip_ratios[0] if self.direction_left else slip_ratios[1]