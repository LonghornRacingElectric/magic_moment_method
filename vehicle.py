from aerodynamics import Aerodynamics
from suspension import Suspension


class Vehicle:
    def __init__(self, suspension, aero):
        self.suspension = suspension
        self.aero = aero