from engine.tire import Tire

class RearTire(Tire):
    def __init__(self, car_params, is_left_tire):
        super().__init__(car_params, is_left_tire)

    # NOT a function of steered angle, don't use for calcs
    def steering_induced_slip(self, steered_angle):
        return self.toe

    @property
    def toe(self):
        return self.params.rear_toe * (1 if self.direction_left else -1) #TODO : verify this

    @property
    def tire_coeffs(self):
        return self.params.rear_tire_spring_coeffs
    
    @property
    def lateral_coeffs(self):
        return self.params.rear_tire_coeff_Fy
    
    @property
    def longitudinal_coeffs(self):
        return self.params.rear_tire_coeff_Fx

    @property
    def arb_stiffness(self): # Nm / rad
        return self.params.rear_arb_stiffness * (1 if self.direction_left else -1)

    # NOTE: Explanation of motion ratios & wheelrates: https://en.wikipedia.org/wiki/Motion_ratio
    @property
    def wheelrate(self): # N/m
        return self.params.rear_spring_springrate/self.params.rear_motion_ratio**2

    @property
    def KPI(self): # rad
        return self.params.rear_KPI

    @property
    def static_camber(self): # rad
        return self.params.rear_static_camber

    @property
    def camber_gain(self): # rad/rad
        return self.params.rear_camber_gain

    @property
    def caster(self): # rad
        return self.params.rear_caster
    
    @property
    def trackwidth(self): # m
        return self.params.rear_track

    @property
    def position(self): # [m, m, m]
        y_pos = self.trackwidth/2 * (1 if self.direction_left else -1)
        return [-self.params.wheelbase * (1 - self.params.cg_bias), y_pos, 0]
    
    # TODO: implement, toe gain near 0 right now though
    def steered_inclination_angle_gain(self, steered_angle):
        return 0

    @property
    def tube_geometry(self):
        return self.params.rear_tube_normals, self.params.rear_lever_arms

    @property
    def radius(self):
        return self.params.rear_tire_radius

    def get_slip_ratio(self, slip_ratios):
        return slip_ratios[2] if self.direction_left else slip_ratios[3]