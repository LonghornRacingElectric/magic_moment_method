from datetime import datetime

import numpy as np
from scipy.optimize import fsolve
from copy import copy
import warnings
import engine
import vehicle_params
import logging
import os


class Solver:

    def __init__(self, vehicle_parameters: vehicle_params.BaseVehicle, initial_guess: dict = None, create_logs: bool = False):
        """_summary_

        Args:
            vehicle_parameters (vehicle_params._parameter_file_): specific static & initial vehicle parameters
            initial_guess (dict, optional): dictionary of 6 solver initial dependent parameter guesses. Defaults to None.
        """
        self.__initial_guess = [initial_guess[x] for x in self.__output_variable_names] if initial_guess else [0, 0, 0,
                                                                                                               0, 0, 0]
        self.vehicle = engine.Vehicle(vehicle_parameters)
        self.create_logs = create_logs
        if create_logs: self.createLogFile()

    def createLogFile(self):
        PATH = datetime.now().strftime('logs\log_%d_%m_%Y')
        i = 0
        while os.path.exists(PATH + "_%s.log" % i):
            i += 1

        logging.basicConfig(filename=PATH + "_%s.log" % i, level=logging.DEBUG,
                            format='%(asctime)s:%(levelname)s:%(message)s')
        logging.debug('NEW SOLVE: ')

    def solve(self, input_state: engine.State):
        """
        Non-linear solve for unique output variable set to match the prescribed states, with an initial guess of dependent states
        Converges on dependent vehicle states by iterating dependent parameter guesses

        Args:
            input_state (engine.State): the independent vehicle states

        Returns:
            dict: data on dependent state variables
        """
        self.vehicle.state = input_state

        # fsolve creates a bunch of annoying warnings, here is a way to filter them if needed
        warnings.filterwarnings('ignore', 'The iteration is not making good progress')

        # allow up to 5 chances for convergence - above 20 doesnt lead to many additional convergences
        guesses_allowed = 5

        for i in range(guesses_allowed):
            results = fsolve(self.__DOF6_motion_residuals, self.__initial_guess, full_output=True)
            if results[2] == 1:
                if i != 0:
                    if self.create_logs: logging.debug("Solution converged after changing initial guess")
                else:
                    pass
                    # NOTE: Solution converged on first guess!
                return copy(self.vehicle.logger.return_log())
            elif results[2] != 1:
                if i == (guesses_allowed - 1):
                    # print(f"Solution convergence not found after {guesses_allowed} guesses for state: {input_state.body_slip} {input_state.s_dot} {input_state.steered_angle}")
                    if self.create_logs: logging.debug("Solution convergence not found after {} guesses for state: "
                                  "{} {} {}"
                                  .format(guesses_allowed, input_state.body_slip, input_state.s_dot,
                                          input_state.steered_angle))
                    # print(results[1]["fvec"],"\n") # for debugging why the solution didnt converge
                    return {}
                self.__initial_guess[self.__output_variable_names.index("heave")] += 0.00125

    @property
    def __output_variable_names(self):
        return ["heave", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]

    def __DOF6_motion_residuals(self, x: list):
        """
        Calculates six residuals for six degree of freedom vehicle, finds dependent state variables based on input guess.
        Used by non-linear solver to converge on dependent vehicle state variables.

        Args:
            x (list[0:5]): the 6 input guesses being iterated - "heave", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"

        Returns:
            list: 6 output residuals - summation of forces in x/y/z and moments about x/y/z
        """
        heave, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x

        # accelerations
        translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
        translation_accelerations_ntb = self.vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)

        # vehicle loads
        yaw_rate = self.vehicle.get_yaw_rate(translation_accelerations_ntb[1])
        forces, moments = self.vehicle.get_loads(roll, pitch, heave, yaw_rate)
        vehicle_forces_ntb = self.vehicle.intermediate_frame_to_ntb_transform(forces)
        vehicle_moments_ntb = self.vehicle.intermediate_frame_to_ntb_transform(moments)

        # TODO: CoG movements due to roll / pitch / heave
        """ a
        NOTE: 1) ~~Kinetic Moments~~
         -  used when summation of moment is about a point that isn't the center of gravity (CoG)
         -  eq. 17-9 pg. 425 in Hibbler Engineering Mechanics Dynamics textbook
         -  Moment = (mass * accel) x radius + inertia * alpha
         
        NOTE: 2) ~~ Yaw Moment ~~
         -  only rotational acceleration being considered is yaw acceleration
         -  which is why the moment isnt transformed to NTB (no roll/pitch accel)
        """
        angular_accelerations = np.array([0, 0, yaw_acceleration])
        kinetic_moments = self.vehicle.get_kinetic_moments(translation_accelerations_ntb, angular_accelerations)
        summation_moments = kinetic_moments - vehicle_moments_ntb

        # F = m * a
        inertial_forces = self.vehicle.get_inertial_forces(translation_accelerations_ntb)
        summation_forces = inertial_forces - vehicle_forces_ntb

        # log dependent states
        [self.vehicle.logger.log(self.__output_variable_names[i], x[i]) for i in range(len(x))]
        self.vehicle.logger.log("vehicle_accelerations_NTB", translation_accelerations_ntb)
        self.vehicle.logger.log("vehicle_kinetic_moment", kinetic_moments)
        self.vehicle.logger.log("vehicle_inertial_forces", inertial_forces)
        self.vehicle.logger.log("vehicle_vehicle_forces_ntb", vehicle_forces_ntb)
        self.vehicle.logger.log("vehicle_yaw_rate", yaw_rate)
        self.vehicle.logger.log("vehicle_x_dot", self.vehicle.x_dot)
        self.vehicle.logger.log("vehicle_y_dot", self.vehicle.y_dot)
        [self.vehicle.logger.log(name, val) for name, val in self.vehicle.state.items()]

        return np.array([*summation_forces, *summation_moments])
