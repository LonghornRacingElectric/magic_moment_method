from ..state_solver.front_tire import FrontTire
from ..state_solver.rear_tire import RearTire
from ..helpers.better_namespace import BetterNamespace

class Tires(BetterNamespace):
    def __init__(self, params):
        self.front_left = FrontTire(params, is_left_tire = True)
        self.front_right = FrontTire(params, is_left_tire = False)
        self.rear_left = RearTire(params, is_left_tire = True)
        self.rear_right = RearTire(params, is_left_tire = False)