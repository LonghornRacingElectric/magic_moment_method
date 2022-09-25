import numpy as np
import engine
import vehicle_params


class Aerodynamics:
    """
    Handles aerodynamic vehicle forces
    """
    def __init__(self, params:vehicle_params.BaseVehicle, logger:engine.Logger):
        self.logger = logger
        # TODO: PUT THESE ALL IN PARAM FILE PLZ
        self.vehicle_params = params # CONTAINS CG POSITION & WHEELBASE LENGTH


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
            Cl_sens = (1 + self.vehicle_params.bs_sens[i][0] * abs(body_slip * self.vehicle_params.rad_to_deg)) * (1 + self.vehicle_params.p_sens[i][0][p_dir] *
                        abs(pitch * self.vehicle_params.rad_to_deg)) * (1 + self.vehicle_params.r_sens[i][0] * abs(roll * self.vehicle_params.rad_to_deg))
            Cd_sens = (1 + self.vehicle_params.bs_sens[i][1] * abs(body_slip * self.vehicle_params.rad_to_deg)) * (1 + self.vehicle_params.p_sens[i][1][p_dir] *
                        abs(pitch * self.vehicle_params.rad_to_deg)) * (1 + self.vehicle_params.r_sens[i][1] * abs(roll * self.vehicle_params.rad_to_deg))
            Cs_sens = (1 + self.vehicle_params.p_sens[i][2][p_dir] * abs(pitch * self.vehicle_params.rad_to_deg)) * (1 + self.vehicle_params.r_sens[i][2] *
                        abs(roll * self.vehicle_params.rad_to_deg))

            # Cl, Cd, and Cs for the compenent
            ClA_part = self.vehicle_params.ClA[i] * Cl_sens
            CdA_part = self.vehicle_params.CdA[i] * Cd_sens
            CsA_part = self.vehicle_params.CsA[i] * Cs_sens

            # calculate force in each direction
            Fl_part = 0.5 * self.__air_density * ClA_part * x_dot ** 2
            Fd_part = 0.5 * self.__air_density * CdA_part * x_dot ** 2
            Fs_part = 0.5 * self.__air_density * CsA_part * (x_dot * np.tan(body_slip)) ** 2 * s_dir

            part_force = np.array([-Fd_part, Fs_part, -Fl_part])
            forces = np.add(forces, part_force)
            moments = np.add(moments, np.cross(self.vehicle_params.CoP[i], part_force))

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
