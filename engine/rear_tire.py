from engine.tire import Tire

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
    def tire_springrate(self):
        return self.params.rear_tire_spring_rate
    
    @property
    def static_normal_load(self):
        return -1 * self.params.gravity * self.params.mass * self.params.cg_bias / 2

    @property
    def lateral_coeffs(self):
        return self.params.rear_tire_coeff_Fy

    @property
    def arb_stiffness(self): # Nm / rad
        return self.params.rear_arb_stiffness * (1 if self.direction_left else -1)

    # NOTE: Explanation of motion ratios & wheelrates: https://en.wikipedia.org/wiki/Motion_ratio
    @property
    def wheelrate(self): # N/m
        return self.params.rear_spring_springrate/self.params.rear_motion_ratio**2

    @property
    def riderate(self): # N/m
        return (self.wheelrate * self.tire_springrate) / (self.wheelrate + self.tire_springrate)

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