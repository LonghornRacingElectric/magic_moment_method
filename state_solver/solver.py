import numpy as np
from scipy.optimize import fsolve
from copy import copy
import warnings
from ..state_solver.state import State
from ..state_solver.vehicle import Vehicle

class Solver:
    def __init__(self, vehicle_parameters):
        """_summary_
        Args:
            vehicle_parameters (vehicle_params._parameter_file_): specific static & initial vehicle parameters
        """
        self.vehicle = Vehicle(vehicle_parameters)

    def solve(self, input_state:State, initial_guess:list = [0.002, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
        """
        Non-linear solve for unique output variable set to match the prescribed states, with an initial guess of dependent states
        Converges on dependent vehicle states by iterating dependent parameter guesses
        Args:
            input_state (engine.State): the independent vehicle states
            initial_guess (list, optional): list of 6 solver initial dependent parameter guesses - 
                "heave", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch". Defaults to 0s.
        Returns:
            dict: data on dependent state variables
        """
        self.vehicle.state = input_state

        # fsolve creates a bunch of annoying warnings, here is a way to filter them if needed
        warnings.filterwarnings('ignore', 'The iteration is not making good progress')

        # NOTE: Sometimes adjusting heave improves convergence after first guess fails
        guesses_allowed = 1
        for i in range(guesses_allowed):
            try:
                results = fsolve(self.__DOF6_motion_residuals, initial_guess, full_output = True)
            except:
                return None
            if results[2] == 1:
                # if i != 0:
                #     print("Solution converged after changing initial guess")
                # else:
                #     print("Solution converged on first guess!")
                break
            elif results[2] != 1:
                if i == (guesses_allowed -1 ):
                    #print(f"Solution convergence not found after {guesses_allowed} guesses for state: {input_state.body_slip} {input_state.s_dot} {input_state.steered_angle}")
                    #print(results[1]["fvec"],"\n") # for debugging why the solution didnt converge
                    return None
                # increment heave to improve convergence
                initial_guess[0] += 0.0008

        # if solution converged, FILTER OUT BAD POINTS
        output = copy(self.vehicle.logger.return_log())
        if output["roll"] > 180/np.pi * 10 or output["yaw_acceleration"] > 200 or output["motor_angular_velocity"] > self.vehicle.params.max_motor_speed:
            return None

        output["power_into_motor"], output["motor_efficiency"] = self.vehicle.motor.power_input_and_efficiency(
                output['motor_torque'], output['motor_angular_velocity'])

        output["power_into_inverter"] = output["power_into_motor"] / .97 # TODO: move this

        if output["power_into_inverter"] > self.vehicle.params.power_limit:
            return None
       
        return output

    def __DOF6_motion_residuals(self, x:list):
        """
        Calculates residuals for six degree of freedom vehicle, finds dependent state variables based on input guess.
        Used by non-linear solver to converge on dependent vehicle state variables.

        Args:
            x (list[0:5]): the 10 input guesses being iterated - "heave", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch", "wheel_speeds"

        Returns:
            list: 10 output residuals - summation of forces in x/y/z, moments about x/y/z, and torques about axles 1/2/3/4
        """
        heave, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch = x[:6]
        translation_accelerations_imf = np.array([x_double_dot, y_double_dot, 0])
        # cap the slip ratios at -100% and 100% (physically possible slip ratios)
        # TODO: implement this in the tire model themselves? this is scrappy AF
        slip_ratios = np.array([])
        for slip in x[6:]:
            if slip < -1:
                slip = -1
            elif slip > 1:
                slip = 1
            slip_ratios = np.append(slip_ratios, [slip])

        # accelerations
        translation_accelerations_ntb = self.vehicle.intermediate_frame_to_ntb_transform(translation_accelerations_imf)

        # vehicle loads
        yaw_rate = self.vehicle.get_yaw_rate(translation_accelerations_ntb[1])
        forces, moments, wheel_angular_velocity, tire_torques = self.vehicle.get_loads(roll, pitch, heave, yaw_rate, slip_ratios)

        vehicle_forces_ntb = self.vehicle.intermediate_frame_to_ntb_transform(forces)
        vehicle_moments_ntb = self.vehicle.intermediate_frame_to_ntb_transform(moments)

        # TODO: CoG movements due to roll / pitch / heave not captured
        """ 
        NOTE: 1) ~~Kinetic Moments~~
         -  used when summation of moment is about a point that isn't the center of gravity (CoG)
         -  eq. 17-9 pg. 425 in Hibbler Engineering Mechanics Dynamics textbook
         -  Moment = (mass * accel) x radius + inertia * alpha
         
        NOTE: 2) ~~ Yaw Moment ~~
         -  only rotational acceleration being considered is yaw acceleration
         -  which is why the moment isnt transformed to NTB (no roll/pitch accel)
        """
        # Summation of Moments = I * alpha
        angular_accelerations = np.array([0, 0, yaw_acceleration])
        kinetic_moments = self.vehicle.get_kinetic_moments(translation_accelerations_ntb, angular_accelerations)
        summation_moments = kinetic_moments - vehicle_moments_ntb
        # Summation of Forces = m * a
        inertial_forces = self.vehicle.get_inertial_forces(translation_accelerations_ntb)
        summation_forces = inertial_forces - vehicle_forces_ntb
        
        ####################################
        #### ~~~ DIFFERENTIAL MODEL ~~~ ####
        # TODO: MOVE ALL THIS STUFF TO A DIFF MODEL
        params = self.vehicle.params
        brake_torques = self.vehicle.brake_request_to_torque(self.vehicle.state.torque_request)
        diff_angular_velocity = sum(wheel_angular_velocity[2:])/len(wheel_angular_velocity[2:])
        motor_angular_velocity = diff_angular_velocity * params.diff_radius / params.motor_radius
        diff_output_torques = tire_torques[2:] - brake_torques[2:]
        motor_torque = 0 if self.vehicle.state.torque_request < 0 else self.vehicle.state.torque_request * self.vehicle.params.max_torque
        force_chain = motor_torque / params.motor_radius
        total_diff_torque = force_chain * params.diff_radius * params.diff_efficiency
        diff_bias_matrix = self.vehicle.torque_bias_ratio(total_diff_torque)
        rear_axle_residuals = diff_bias_matrix * np.array([total_diff_torque, total_diff_torque]) - diff_output_torques
        front_axle_residuals = tire_torques[:2] - brake_torques[:2]

        ####################################
        # log all states
        self.vehicle.logger.log("heave", heave)
        self.vehicle.logger.log("x_double_dot", x_double_dot)
        self.vehicle.logger.log("y_double_dot", y_double_dot)
        self.vehicle.logger.log("yaw_acceleration", yaw_acceleration)
        self.vehicle.logger.log("roll", roll)
        self.vehicle.logger.log("pitch", pitch)
        self.vehicle.logger.log("front_left_tire_slip_ratio", slip_ratios[0])
        self.vehicle.logger.log("front_right_tire_slip_ratio", slip_ratios[1])
        self.vehicle.logger.log("rear_left_tire_slip_ratio", slip_ratios[2])
        self.vehicle.logger.log("rear_right_tire_slip_ratio", slip_ratios[3])
        self.vehicle.logger.log("motor_angular_velocity", motor_angular_velocity)
        self.vehicle.logger.log("motor_torque", motor_torque)
        self.vehicle.logger.log("brake_torques", brake_torques)
        self.vehicle.logger.log("vehicle_accelerations_NTB", translation_accelerations_ntb)
        self.vehicle.logger.log("vehicle_kinetic_moment", kinetic_moments)
        self.vehicle.logger.log("vehicle_inertial_forces", inertial_forces)
        self.vehicle.logger.log("vehicle_vehicle_forces_ntb", vehicle_forces_ntb) 
        self.vehicle.logger.log("vehicle_yaw_rate", yaw_rate)
        self.vehicle.logger.log("vehicle_x_dot", self.vehicle.x_dot)
        self.vehicle.logger.log("vehicle_y_dot", self.vehicle.y_dot)
        self.vehicle.logger.log("body_slip", self.vehicle.state.body_slip)
        self.vehicle.logger.log("steered_angle", self.vehicle.state.steered_angle)
        self.vehicle.logger.log("s_dot", self.vehicle.state.s_dot)
        self.vehicle.logger.log("torque_request", self.vehicle.state.torque_request)
        self.vehicle.logger.log("is_left_diff_bias", self.vehicle.state.is_left_diff_bias)

        return np.array([*summation_forces, *summation_moments, *front_axle_residuals, *rear_axle_residuals])