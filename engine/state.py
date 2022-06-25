from helpers.better_namespace import BetterNamespace


# these are the prescribed MMM states!
# other states are solved for to match these conditions essentially :)
class State(BetterNamespace):
    
    def __init__(self, body_slip, steered_angle, s_dot):
        self.body_slip = body_slip # rad
        self.steered_angle = steered_angle # rad
        self.s_dot = s_dot # m/s # NOTE: vehicle momentum forwards in NTB(?) verify
    