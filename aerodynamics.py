import math
import numpy as np

class Aerodynamics:
    def __init__(self, params):
        # TODO: make param files later; currently all initialized here
        self.vehicle_params = params # CONTAINS CG POSITION & WHEELBASE LENGTH

        # distribution of downforce across components
        ClA_dist = [[0.440, 0.184, 0.376], [0.454, 0.407, 0.139]]   # [active, deactive] -> [front, undertray, rear]
        CdA_dist = [[0.415, 0.150, 0.445], [0.426, 0.404, 0.170]]
        CsA_dist = [[0.250, 0.000, 0.750], [0.250, 0.000, 0.750]]

        # pitch, body_slip, and roll sensitivities,
        #            Cl              Cd               Cs
        self.p_sens	= [[[-.157, -.106], [-.075,  -.125],  [0,0]], # front [pos, neg] -> [%/deg]
                      [[.0076, -.0457], [.0546, -.0434],  [0,0]], # undertray
                      [[-.0072, -.0406], [.0202, -.0594], [0,0]]]  # rear

        #                 front               undertray           rear
        self.bs_sens	= [[-.024,  -.007, 0], [.0061,  .0089, 0], [-.0096, .0033, 0]] # [Cl, Cd, Cs] -> [%/deg]
        self.r_sens	=     [[-.101,  -0.131,    0], [-.0137,  0,    0], [-.0424,  -.0202, 0]]

        # conversion factors
        self.in_to_m = 0.0254
        self.rad_to_deg = 180 / math.pi

        # positions of component CoPs (magnitudes, equation takes signs into account)
        # TODO: make positions relative to intermediate frame
        # Front, Undertray and Rear [x , y , z]
        self.CoP = np.array([[23.71 * self.in_to_m + self.vehicle_params.cg_total_position[0],  0, 6.6 * self.in_to_m],
                             [-63.52 * self.in_to_m + self.vehicle_params.cg_total_position[0],  0, 14.85 * self.in_to_m],
                             [-65.16 * self.in_to_m + self.vehicle_params.cg_total_position[0],  0, 37.84 * self.in_to_m]])

        # sets values for active/deactive aero
        if self.vehicle_params.aeroActive:
            distrArray = 0
        else:
            distrArray = 1

        # gets aero coefficients for each component
        self.ClA = [self.vehicle_params.ClA_tot * ClA_dist[distrArray][0], self.vehicle_params.ClA_tot * ClA_dist[distrArray][1],
                    self.vehicle_params.ClA_tot * ClA_dist[distrArray][2]]
        self.CdA = [self.vehicle_params.CdA_tot * CdA_dist[distrArray][0], self.vehicle_params.CdA_tot * CdA_dist[distrArray][1],
                    self.vehicle_params.CdA_tot * CdA_dist[distrArray][2]]
        self.CsA = [self.vehicle_params.CsA_tot * CsA_dist[distrArray][0], self.vehicle_params.CsA_tot * CsA_dist[distrArray][1],
                    self.vehicle_params.CsA_tot * CsA_dist[distrArray][2]]

    def get_loads(self, x_dot, body_slip, pitch, roll, rideheight):

        forces  = np.array([0, 0, 0])
        moments = np.array([0, 0, 0])

        p_dir = pitch <= 0
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
            Fl_part = 0.5 * 1.225 * ClA_part * x_dot ** 2
            Fd_part = 0.5 * 1.225 * CdA_part * x_dot ** 2
            Fs_part = 0.5 * 1.225 * CsA_part * (x_dot * math.tan(body_slip)) ** 2 * s_dir

            part_force = np.array([-Fd_part, Fs_part, -Fl_part])
            forces = np.add(forces, part_force)
            moments = np.add(moments, np.cross(self.CoP[i], part_force))

        # account for drag from rest of car
        drag_no_aero = 0.5 * 1.225 * self.vehicle_params.CdA0 * x_dot ** 2
        forces[0] -= drag_no_aero
        return forces, moments
