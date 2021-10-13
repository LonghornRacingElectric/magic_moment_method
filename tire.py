import math
import numpy as np

"""
Coordinate Systems:
    SAE z-up wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html

Units: 
    Length: meters
    Mass: kg
"""
class Tire:
    def __init__(self, pacejka_fit_params, location, stiffness, direction_left):
        # Pacejka Coefficients, specific to each tire
        self.pacejka_fit = PacejkaFit(*pacejka_fit_params)
        self.stiffness = stiffness
        self.direction_left = direction_left # Boolean
        self.position = np.array(location)  # Position of the center of the contact patch relative to vehicle frame
        self.unloaded_radius = 9 * 0.0254

        self.unsprung_displacement = None

    @property
    def normal_load(self):
        return self.stiffness*self.unsprung_displacement

    # Determines the lateral force on the tire given the pacejka fit coefficients, slip angle, camber, and normal load
    # https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
    def get_force(self, normal_force, slip_angle, inclination_angle, slip_ratio = None):
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16,
         a17] = self.param.pacejka_fit.Fy_coefficients
        Fz = self.state.force[2]

        C = a0
        D = Fz * (a1 * Fz + a2) * (1 - a15 * self.state.camber ** 2)
        BCD = a3 * math.sin(math.atan(Fz / a4) * 2) * (1 - a5 * abs(self.state.camber))
        B = BCD / (C * D)
        H = a8 * Fz + a9 + a10 * self.state.camber
        E = (a6 * Fz + a7) * (1 - (a16 * self.state.camber + a17) * math.copysign(1, self.state.slip_angle + H))
        V = a11 * Fz + a12 + (a13 * Fz + a14) * self.state.camber * Fz
        Bx1 = B * (self.state.slip_angle + H)

        Fy = D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V

        return Fy * (1 if self.direction_left else -1)


    # def get_Fx(self, slip_angle, camber, Fz):
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

    def get_cornering_stiffness(self): # Cornering stiffness is Fy/slip_angle
        return

# Stores predetermined pacejka fits for several tires
class PacejkaFit:
    def __init__(self, Fx_coefficients=None,
                 Fy_coefficients=None,
                 Mz_coefficients=None):
        self.Fx_coefficients = Fx_coefficients
        self.Fy_coefficients = Fy_coefficients
        self.Mz_coefficients = Mz_coefficients
