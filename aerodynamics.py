import math
import numpy as np

class Aerodynamics:
    def __init__(self):
        # TODO: make param files later; currently all initialized here
        
        #parameters
        ClA_tot = 3.955
        self.CdA0    = 0.7155
        CdA_tot = 1.512 - self.CdA0
        CsA_tot = 33.91

        # distribution of downforce across components
        ClA_dist = [0.371, 0.282, 0.347]   # [front, undertray, rear]
        CdA_dist = [0.324, 0.186, 0.490]
        CsA_dist = [0.250, 0.000, 0.750]

        # pitch, bodyslip, and roll sensitivities,
        #            Cl              Cd               Cs
        self.p_sens	= [[[.01,   -.06],   [.07,   -.055],  [0,0]], # front [pos, neg] -> [%/deg]
                [[.0076, -.0457], [.0546, -.0434], [0,0]], # undertray
                [[.0178, -.0245], [.0294, -.0478], [0,0]]] # rear

        #          front              undertray           rear
        self.bs_sens	= [[.008,  .0114, 0], [.0061,  .0089, 0], [-.0018, -.0058, 0]] # [Cl, Cd, Cs] -> [%/deg]
        self.r_sens	= [[-.018,  0,    0], [-.0137,  0,    0], [-.005,  -.0145, 0]]

        self.in_to_m = 0.0254 # conversion factor

        # positions of component CoPs (magnitudes, equation takes signs into account)
        # TODO: make positions relative to intermediate frame
        self.CoP = np.array([[23.65 * self.in_to_m, 0, 9.30 * self.in_to_m], [43.5 * self.in_to_m, 0, 7.13 * self.in_to_m], [67.6 * self.in_to_m, 0, 42.91 * self.in_to_m]])    

        # gets aero coefficients for each component
        self.ClA = [ClA_tot * ClA_dist[0], ClA_tot * ClA_dist[1], ClA_tot * ClA_dist[2]]
        self.CdA = [CdA_tot * CdA_dist[0], CdA_tot * CdA_dist[1], CdA_tot * CdA_dist[2]]
        self.CsA = [CsA_tot * CsA_dist[0], CsA_tot * CsA_dist[1], CsA_tot * CsA_dist[2]]

    def get_loads(self, velocity, bodyslip, pitch, roll, rideheight):

        forces = np.array([0, 0, 0])
        moments = np.array([0, 0, 0])

        p_dir = pitch <= 0

        # iterates for each aero component
        for i in range (0, 3):

            # sensitivities for Cl, Cd, and Cs
            Cl_sens = (1 + self.bs_sens[i][0] * bodyslip) * (1 + self.p_sens[i][0][p_dir] * pitch) * (
                    1 + self.r_sens[i][0] * roll)
            Cd_sens = (1 + self.bs_sens[i][1] * bodyslip) * (1 + self.p_sens[i][1][p_dir] * pitch) * (
                    1 + self.r_sens[i][1] * roll)
            Cs_sens = (1 + self.p_sens[i][2][p_dir] * pitch) * (1 + self.r_sens[i][2] * roll)

            # Cl, Cd, and Cs for the compenent
            ClA_part = self.ClA[i] * Cl_sens
            CdA_part = self.CdA[i] * Cd_sens
            CsA_part = self.CsA[i] * Cs_sens

            # calculate force in each direction
            Fl_part = 0.5 * 1.225 * ClA_part * velocity ** 2
            Fd_part = 0.5 * 1.225 * CdA_part * velocity ** 2
            Fs_part = 0.5 * 1.225 * CsA_part * (velocity * math.sin(bodyslip)) ** 2

            part_force = np.array([-Fd_part, Fs_part, -Fl_part])
            forces = np.add(forces, part_force)
            moments = np.add(moments, np.cross(part_force, self.CoP[i]))

        # account for drag from rest of car
        # TODO: Account for moment for this drag
        drag_no_aero = 0.5 * 1.225 * self.CdA0 * velocity ** 2
        forces[0] -= drag_no_aero

        return forces, moments

    '''
    results = calc_CoP(30, 3, 0, 0, 0)

    forces = results[0]
    moment = results[1]

    np.set_printoptions(suppress = True)
    np.set_printoptions(precision = 2)

    print("Total forces [x,y,z]: {:.2f} N, {:.2f} N, {:.2f} N".format(forces[0], forces[1], forces[2]))
    print("Total moment        : {:.2f} Nm".format(moment))
    '''