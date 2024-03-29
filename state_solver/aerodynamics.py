import numpy as np
from ..state_solver.logger import Logger

class Aerodynamics:
    """
    Handles aerodynamic vehicle forces
    """
    def __init__(self, params, logger:Logger):
        self.logger = logger
        self.vehicle_params = params

        # gets aero coefficients for each component
        self.ClA = self.vehicle_params.ClA_tot * self.vehicle_params.ClA_dist
        self.CdA = self.vehicle_params.CdA_tot * self.vehicle_params.CdA_dist
        self.CsA = self.vehicle_params.CsA_tot * self.vehicle_params.CsA_dist

        # converts from CAD origin to IMF
        self.vehicle_params.CoP_IMF = self.vehicle_params.CoP
        self.vehicle_params.CoP_IMF[:,0] += self.vehicle_params.cg_bias * self.vehicle_params.wheelbase


    def get_loads(self, x_dot, body_slip, pitch, roll, heave):
        # TODO: Add heave sensitivities
        # TODO: Add CoP position to log

        # conversion factors
        rad_to_deg = 180 / np.pi

        # converts angles to degrees for sensitivity calcs
        body_slip *= rad_to_deg
        pitch *= rad_to_deg
        roll *= rad_to_deg

        coefs = np.array([self.ClA, self.CdA, self.CsA])

        p_dir = 1 if pitch <= 0 else 0
        psens = self.vehicle_params.p_sens[:,:,p_dir]

        angles = np.array([abs(body_slip), abs(pitch), abs(roll)])
        angle_sens = np.array([self.vehicle_params.bs_sens, psens, self.vehicle_params.r_sens])
        angle_sens /= 100   # convert from percentages

        s_dir = -1
        if body_slip < 0:
            s_dir = 1

        # multiply each sensitivity by the corresponding angle
        # repeat -> reshape turns angles array into 3x3x3 array with angles in
        #   the right place for element-wise multiplication
        angle_sens *= np.reshape(np.repeat(angles,9),(3,3,3))

        # add 1 to turn percent increases into multiplication factor
        angle_sens += 1

        # multiply all sensitivities for given part and force together
        angle_sens = np.prod(angle_sens, axis = 0)

        # multiply lift, drag, and sideforce coefficients by sensitivities
        coefs = angle_sens * coefs.T

        # heave sensitivities
        heave_sens = self.get_heave_sens(heave)
        coefs *= heave_sens

        # calculate force arrays for each direction: F_part = [front, undertray, rear]
        Fl_part = 0.5 * self.__air_density * coefs[:,0] * x_dot ** 2
        Fd_part = 0.5 * self.__air_density * coefs[:,1] * x_dot ** 2
        Fs_part = 0.5 * self.__air_density * coefs[:,2] * (x_dot * np.tan(body_slip/rad_to_deg)) ** 2 * s_dir

        # drag is x, sideforce is y, lift is z
        part_force = np.array([-Fd_part, Fs_part, -Fl_part])

        # forces are sum of forces on each part
        forces = np.array([-np.sum(Fd_part), np.sum(Fs_part), -np.sum(Fl_part)])

        # sum moments from front, undertray, and rear
        moments = np.cross(self.vehicle_params.CoP_IMF[0], part_force.T[0]) \
                + np.cross(self.vehicle_params.CoP_IMF[1], part_force.T[1]) \
                + np.cross(self.vehicle_params.CoP_IMF[2], part_force.T[2])


        # account for drag and sideforce from rest of car
        drag_no_aero = 0.5 * self.__air_density * self.vehicle_params.CdA0 * x_dot ** 2
        sideforce_no_aero = 0.5 * self.__air_density * self.vehicle_params.CsA0 * (x_dot * np.tan(body_slip/rad_to_deg)) ** 2 * s_dir
        forces += np.array([-drag_no_aero, sideforce_no_aero, 0])

        self.logger.log("aero_forces", forces)
        self.logger.log("aero_moments", moments)

        # cop_x = moments[1]/(((forces[0]**2 + forces[2]**2))**(1/2)) * np.cos( np.arctan( forces[2] / forces[0])) 
        # cop_y = moments[2]/(((forces[0]**2 + forces[2]**2))**(1/2)) * np.cos( np.arctan( forces[2] / forces[0]))
        # self.logger.log("aero_cop_x",cop_x)
        # self.logger.log("aero_cop_y",cop_y)

        return forces, moments

    def get_heave_sens(self, heave):
       cl_heave_sens = np.polyval(self.vehicle_params.h_sens_coefficients[0], heave)
       cd_heave_sens = np.polyval(self.vehicle_params.h_sens_coefficients[1], heave)

       # sens for undertraying, using it as an estimate for front wing
       heave_sens = np.array([[cl_heave_sens,cd_heave_sens,1],
                              [cl_heave_sens,cd_heave_sens,1],
                              [1,1,1]])

       return heave_sens
    
    # NOTE: Linear assumption data reference https://en.wikipedia.org/wiki/Density_of_air
    @property
    def __air_density(self): # kg/m^3
        return  1.225 - 0.003975 * (self.vehicle_params.air_temperature - 15)
