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

        # conversion factors
        in_to_m = 0.0254

        # gets aero coefficients for each component
        self.ClA = self.vehicle_params.ClA_tot * self.vehicle_params.ClA_dist
        self.CdA = self.vehicle_params.CdA_tot * self.vehicle_params.CdA_dist
        self.CsA = self.vehicle_params.CsA_tot * self.vehicle_params.CsA_dist

        self.CoP = self.vehicle_params.CoP * in_to_m
        self.CoP[:,0] += self.vehicle_params.cg_bias * self.vehicle_params.wheelbase


    def get_loads(self, x_dot, body_slip, pitch, roll, heave):

        moments = np.array([0, 0, 0])

        rad_to_deg = 180 / np.pi

        # converts angles to degrees for sensitivity calcs
        body_slip *= rad_to_deg
        pitch *= rad_to_deg
        roll *= rad_to_deg

        coefs  = np.array([self.ClA, self.CdA, self.CsA])

        p_dir = 1 if pitch <= 0 else 0
        psens = self.vehicle_params.p_sens[:,:,p_dir]

        angles = np.array([abs(body_slip), abs(pitch), abs(roll)])
        angle_sens = np.array([self.vehicle_params.bs_sens, psens, self.vehicle_params.r_sens])


        s_dir = -1
        if body_slip < 0:
            s_dir = 1

        # multiply each sensitivity by the corresponding angle
        # repeat -> reshape turns angles array into 3x3x3 array with angles in
        #   the right place for element-wise multiplication
        angle_sens = np.multiply(angle_sens, np.reshape(np.repeat(angles,9),(3,3,3)))

        # add 1 to turn percent increases into multiplication factor
        angle_sens += 1

        # multiply all sensitivities for given part and force together
        angle_sens = np.prod(angle_sens, axis = 0)

        # multiply lift, drag, and sideforce coefficients by sensitivities
        coefs = np.multiply(angle_sens, coefs.T)

        # calculate force arrays for each direction: F_part = [front, undertray, rear]
        Fl_part = 0.5 * self.__air_density * coefs[:,0] * x_dot ** 2
        Fd_part = 0.5 * self.__air_density * coefs[:,1] * x_dot ** 2
        Fs_part = 0.5 * self.__air_density * coefs[:,2] * (x_dot * np.tan(body_slip/rad_to_deg)) ** 2 * s_dir

        # drag is x, sideforce is y, lift is z
        part_force = np.array([-Fd_part, Fs_part, -Fl_part])
        # forces are sum of forces on each part
        forces = np.array([-np.sum(Fd_part), np.sum(Fs_part), -np.sum(Fl_part)])
        # sum moments from front, undertray, and rear
        moments = np.cross(self.CoP[0], part_force.T[0]) \
                + np.cross(self.CoP[1], part_force.T[1]) \
                + np.cross(self.CoP[2], part_force.T[2])


        # account for drag and sideforce from rest of car
        drag_no_aero = 0.5 * self.__air_density * self.vehicle_params.CdA0 * x_dot ** 2
        forces[0] -= drag_no_aero
        sideforce_no_aero = 0.5 * self.__air_density * self.vehicle_params.CsA0 * (x_dot * np.tan(body_slip/rad_to_deg)) ** 2 * s_dir
        forces[1] += sideforce_no_aero

        self.logger.log("aero_forces", forces)
        self.logger.log("aero_moments", moments)

        return forces, moments

    # NOTE: Linear assumption data reference https://en.wikipedia.org/wiki/Density_of_air
    @property
    def __air_density(self): # kg/m^3
        return  1.225 - 0.003975 * (self.vehicle_params.air_temperature - 15)
