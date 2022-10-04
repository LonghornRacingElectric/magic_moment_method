import numpy as np
import engine


class Aerodynamics:
    """
    Handles aerodynamic vehicle forces
    """
    def __init__(self, params, logger:engine.Logger):
        self.logger = logger
        # TODO: PUT THESE ALL IN PARAM FILE PLZ
        self.vehicle_params = params # CONTAINS CG POSITION & WHEELBASE LENGTH

        # distribution of downforce across components
        ClA_dist = [0.474, 0.289, 0.236]   # [front, undertray, rear]
        CdA_dist = [0.425, 0.178, 0.396]
        CsA_dist = [0.250, 0.000, 0.750]

        # pitch, body_slip, and roll sensitivities,
        #            Cl              Cd               Cs
        self.p_sens	= [[[.01,   -.06],   [.07,   -.055],  [0,0]], # front [pos, neg] -> [%/deg]
                      [[.0076, -.0457], [.0546, -.0434],  [0,0]], # undertray
                      [[.0178, -.0245], [.0294, -.0478], [0,0]]]  # rear

        #                 front               undertray           rear
        self.bs_sens	= [[.008,  .0114, 0], [.0061,  .0089, 0], [-.0018, -.0058, 0]] # [Cl, Cd, Cs] -> [%/deg]
        self.r_sens	=     [[-.018,  0,    0], [-.0137,  0,    0], [-.005,  -.0145, 0]]

        # conversion factors
        self.in_to_m = 0.0254
        self.rad_to_deg = 180 / np.pi

        # positions of component CoPs (magnitudes, equation takes signs into account)
        # TODO: make positions relative to intermediate frame; these lines also seem wrong in general ATM
        # Front, Undertray and Rear [x , y , z]
        self.CoP = np.array([[23.65 * self.in_to_m + self.vehicle_params.cg_total_position[0],  0, 9.30 * self.in_to_m],
                             [-43.5 * self.in_to_m + self.vehicle_params.cg_total_position[0],  0, 7.13 * self.in_to_m],
                             [-67.6 * self.in_to_m + self.vehicle_params.cg_total_position[0],  0, 42.91 * self.in_to_m]])

        # gets aero coefficients for each component
        self.ClA = [self.vehicle_params.ClA_tot * ClA_dist[0], self.vehicle_params.ClA_tot * ClA_dist[1],
                    self.vehicle_params.ClA_tot * ClA_dist[2]]
        self.CdA = [self.vehicle_params.CdA_tot * CdA_dist[0], self.vehicle_params.CdA_tot * CdA_dist[1],
                    self.vehicle_params.CdA_tot * CdA_dist[2]]
        self.CsA = [self.vehicle_params.CsA_tot * CsA_dist[0], self.vehicle_params.CsA_tot * CsA_dist[1],
                    self.vehicle_params.CsA_tot * CsA_dist[2]]

    def get_loads(self, x_dot, body_slip, pitch, roll, heave):

        forces  = np.array([0, 0, 0])
        moments = np.array([0, 0, 0])

        p_dir = 1 if pitch <= 0 else 0
        s_dir = -1
        if body_slip < 0:
            s_dir = 1

        # iterates for each aero component
        for i in range (0, 3):

            # sensitivities for Cl, Cd, and Cs
            Cl_sens = (1 + self.bs_sens[i][0] * abs(body_slip * self.rad_to_deg)) * (1 + self.p_sens[i][0][p_dir] *
                        abs(pitch * self.rad_to_deg)) * (1 + self.r_sens[i][0] * abs(roll * self.rad_to_deg))
            Cd_sens = (1 + self.bs_sens[i][1] * abs(body_slip * self.rad_to_deg)) * (1 + self.p_sens[i][1][p_dir] *
                        abs(pitch * self.rad_to_deg)) * (1 + self.r_sens[i][1] * abs(roll * self.rad_to_deg))
            Cs_sens = (1 + self.p_sens[i][2][p_dir] * abs(pitch * self.rad_to_deg)) * (1 + self.r_sens[i][2] *
                        abs(roll * self.rad_to_deg))

            # Cl, Cd, and Cs for the compenent
            ClA_part = self.ClA[i] * Cl_sens
            CdA_part = self.CdA[i] * Cd_sens
            CsA_part = self.CsA[i] * Cs_sens

            # calculate force in each direction
            Fl_part = 0.5 * self.__air_density * ClA_part * x_dot ** 2
            Fd_part = 0.5 * self.__air_density * CdA_part * x_dot ** 2
            Fs_part = 0.5 * self.__air_density * CsA_part * (x_dot * np.tan(body_slip)) ** 2 * s_dir

            part_force = np.array([-Fd_part, Fs_part, -Fl_part])
            forces = np.add(forces, part_force)
            moments = np.add(moments, np.cross(self.CoP[i], part_force))

        # account for drag from rest of car
        drag_no_aero = 0.5 * self.__air_density * self.vehicle_params.CdA0 * x_dot ** 2
        forces[0] -= drag_no_aero
        
        self.logger.log("aero_forces", forces)
        self.logger.log("aero_moments", moments)
        
        return forces, moments

    # NOTE: Linear assumption data reference https://en.wikipedia.org/wiki/Density_of_air
    @property
    def __air_density(self): # kg/m^3
        return  1.225 - 0.003975 * (self.vehicle_params.air_temperature - 15)
