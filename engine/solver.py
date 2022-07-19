import numpy as np
from scipy.optimize import fsolve as josie_solver
from copy import copy
import warnings
import engine
import vehicle_params

class Solver:
    def __init__(self, vehicle_parameters:vehicle_params.BaseVehicle, initial_guess:dict = None):
        """_summary_

        Args:
            vehicle_parameters (vehicle_params._parameter_file_): specific static & initial vehicle parameters
            initial_guess (dict, optional): dictionary of 6 solver initial dependent parameter guesses. Defaults to None.
        """
        self.default_guess["ride_height"] = vehicle_parameters.static_ride_height
        guess_dict = initial_guess if initial_guess else self.default_guess
        self.initial_guess = [guess_dict[x] for x in Solver.output_variable_names()]
        self.vehicle = engine.Vehicle(vehicle_parameters)


    def solve(self, input_state:engine.State):
        """
        Non-linear solve for unique output variable set to match the prescribed states, with an initial guess of dependent states
        Converges on dependent vehicle states by iterating dependent parameter guesses

        Args:
            input_state (engine.State): the independent vehicle states

        Returns:
            dict: data on dependent state variables
        """
        self.vehicle.state = input_state
        specific_residual_func = lambda x: self.DOF6_motion_residuals(x)

        # fsolve creates a bunch of annoying warnings, here is a way to filter them if needed
        warnings.filterwarnings('ignore', 'The iteration is not making good progress')

        # allow up to 5 chances for convergence
        guesses_allowed = 20
        for i in range(guesses_allowed):
            results  = josie_solver(specific_residual_func, self.initial_guess, full_output = True)
            if results[2] == 1:
                if i != 0:
                    print("Solution converged after changing initial guess")
                else:
                    pass
                    #print("Solution converged on first guess")
                return copy(self.vehicle.logger.return_log())
            elif results[2] != 1:
                if i == (guesses_allowed -1 ):
                    print(f"Solution convergence not found after {guesses_allowed} guesses for state: {input_state.body_slip} {input_state.s_dot} {input_state.steered_angle}")
                    #print(results[1]["fvec"],"\n")
                    return {}
                self.initial_guess[Solver.output_variable_names().index("ride_height")] -= 0.0025

    def DOF6_motion_residuals(self, x:list):
        """
        Calculates six residuals for six degree of freedom vehicle, finds dependent state variables based on input guess.
        Used by non-linear solver to converge on dependent vehicle state variables.

        Args:
            x (list[0:5]): the 6 input guesses being iterated - "ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"

        Returns:
            list: 6 output residuals - summation of forces in x/y/z and moments about x/y/z
        """
        ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x

        # accelerations
        translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
        translation_accelerations_ntb = self.vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)

        # vehicle loads
        yaw_rate = self.vehicle.get_yaw_rate(translation_accelerations_ntb[1])
        forces, moments = self.vehicle.get_loads(roll, pitch, ride_height, yaw_rate)
        vehicle_forces_ntb = self.vehicle.intermediate_frame_to_ntb_transform(forces)
        vehicle_moments_ntb = self.vehicle.intermediate_frame_to_ntb_transform(moments)

        # Kinetic moment summation of moments not being done about CG
        # TODO: Make sure sprung inertia is about the intermediate axis
        # TODO: CoG movements
        kinetic_moments = self.vehicle.get_kinetic_moments(translation_accelerations_ntb) 
        
        # solving for summation of forces = m * accel
        inertial_forces = self.vehicle.get_inertial_forces(translation_accelerations_ntb)
        summation_forces = inertial_forces - vehicle_forces_ntb
        
        # solving for summation of moments = I * alpha
        # NOTE: only rotational acceleration being considered is yaw acceleration
        #       which is why the moment isnt transformed to NTB (no roll/pitch accel)
        inertial_moments = np.array([0, 0, self.vehicle.get_yaw_moment(yaw_acceleration)])
        summation_moments = inertial_moments - kinetic_moments - vehicle_moments_ntb

        # log dependent states
        [self.vehicle.logger.log(Solver.output_variable_names()[i], x[i]) for i in range(len(x))]
        self.vehicle.logger.log("vehicle_accelerations_NTB", translation_accelerations_ntb)
        self.vehicle.logger.log("vehicle_yaw_moment", inertial_moments[2])
        self.vehicle.logger.log("vehicle_kinetic_moment", kinetic_moments)
        self.vehicle.logger.log("vehicle_inertial_forces", inertial_forces)
        self.vehicle.logger.log("vehicle_vehicle_forces_ntb", vehicle_forces_ntb) 
        self.vehicle.logger.log("vehicle_yaw_rate", yaw_rate)
        self.vehicle.logger.log("vehicle_x_dot", self.vehicle.x_dot)
        self.vehicle.logger.log("vehicle_y_dot", self.vehicle.y_dot)
        [self.vehicle.logger.log(name, val) for name, val in self.vehicle.state.items()]

        return np.array([*summation_forces, *summation_moments])


    @property
    def default_guess(self):
        return {"ride_height": 0.0762, "x_double_dot": 0, "y_double_dot": 0, "yaw_acceleration":0,
                    "roll": 0, "pitch": 0}
    
    
    def output_variable_names():
        return ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]