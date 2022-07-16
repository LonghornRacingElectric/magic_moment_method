from helpers.better_namespace import BetterNamespace
import engine


class Tires(BetterNamespace):
    def __init__(self, params):
        self.front_left = engine.FrontTire(params, is_left_tire = True)
        self.front_right = engine.FrontTire(params, is_left_tire = False)
        self.rear_left = engine.RearTire(params, is_left_tire = True)
        self.rear_right = engine.RearTire(params, is_left_tire = False)