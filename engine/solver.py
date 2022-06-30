import numpy as np
from scipy.optimize import fsolve as josie_solver
from copy import copy
import warnings
import engine

class Solver:
    def __init__(self, vehicle_parameters, initial_guess = None):
<<<<<<< HEAD
=======
        # These are the output variables being solved for to match the prescribed states!
>>>>>>> main
        self.parameter_order = ["ride_height", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch"]
        guess_dict = initial_guess if initial_guess else self.default_guess
        self.initial_guess = [guess_dict[x] for x in self.parameter_order]
        self.vehicle = engine.Vehicle(vehicle_parameters)

    # solve for unique output variable set to match the prescribed states, with an initial guess of outputs
    def solve(self, input_state):
        self.vehicle.state = input_state
        specific_residual_func = lambda x: Solver.DOF6_motion_residuals(x, self.vehicle, self.parameter_order)
        warnings.filterwarnings('ignore', 'The iteration is not making good progress')
        josie_solver(specific_residual_func, self.initial_guess)
        return copy(self.vehicle.logger.return_log())

    def DOF6_motion_residuals(x, vehicle, output_var_labels):
        # solving for these bois
        ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x
        
        # accelerations
        translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
        translation_accelerations_ntb = vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)

        # vehicle loads
        yaw_rate = vehicle.get_yaw_rate(translation_accelerations_ntb[1])
        forces, moments = vehicle.get_loads(roll, pitch, ride_height, yaw_rate)
        vehicle_forces_ntb = vehicle.intermediate_frame_to_ntb_transform(forces)
        vehicle_moments_ntb = vehicle.intermediate_frame_to_ntb_transform(moments)

        # Kinetic moment summation of moments not being done about CG
        # TODO: Make sure sprung inertia is about the intermediate axis
        # TODO: CoG movements
        kinetic_moments = vehicle.get_kinetic_moments(translation_accelerations_ntb) 
        
        # solving for summation of forces = m * accel
        inertial_forces = vehicle.get_inertial_forces(translation_accelerations_ntb)
        summation_forces = inertial_forces - vehicle_forces_ntb
        
        # solving for summation of moments = I * alpha
        # only rotational acceleration being considered is yaw acceleration; which is why it isnt transformed (no roll/pitch accel)
<<<<<<< HEAD
        inertial_moments = np.array([0, 0, vehicle.get_yaw_moment(yaw_acceleration)])
        summation_moments = inertial_moments - kinetic_moments - vehicle_moments_ntb
=======
        yaw_moment = vehicle.get_yaw_moment(yaw_acceleration)
        summation_moments = np.array([0, 0, yaw_moment]) - kinetic_moments - vehicle_moments_ntb
>>>>>>> main

        # LOG SOME SHITS
        [vehicle.logger.log(output_var_labels[i], x[i]) for i in range(len(x))]
        vehicle.logger.log("vehicle_accelerations_NTB", translation_accelerations_ntb)
<<<<<<< HEAD
        vehicle.logger.log("vehicle_yaw_moment", inertial_moments[2])
=======
        vehicle.logger.log("vehicle_yaw_moment", yaw_moment)
>>>>>>> main
        vehicle.logger.log("vehicle_kinetic_moment", kinetic_moments)
        vehicle.logger.log("vehicle_inertial_forces", inertial_forces)
        vehicle.logger.log("vehicle_yaw_rate", yaw_rate)
        vehicle.logger.log("vehicle_x_dot", vehicle.x_dot)
        vehicle.logger.log("vehicle_y_dot", vehicle.y_dot)
        [vehicle.logger.log(name, val) for name, val in vehicle.state.items()]

        return np.array([*summation_forces, *summation_moments])

    @property
    def default_guess(self):
        return {"ride_height": 0.0762, "x_double_dot": 0, "y_double_dot": 0, "yaw_acceleration":0,
                    "roll": 0, "pitch": 0}