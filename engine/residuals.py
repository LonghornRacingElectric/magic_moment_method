import numpy as np

class Residuals:
    
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
        yaw_moment = vehicle.get_yaw_moment(yaw_acceleration)
        summation_moments = np.array([0, 0, yaw_moment]) - kinetic_moments - vehicle_moments_ntb

        # LOG SOME SHITS
        [vehicle.logger.log(output_var_labels[i], x[i]) for i in range(len(x))]
        vehicle.logger.log("vehicle_accelerations_NTB", translation_accelerations_ntb)
        vehicle.logger.log("vehicle_yaw_moment", yaw_moment)
        vehicle.logger.log("vehicle_kinetic_moment", kinetic_moments)
        vehicle.logger.log("vehicle_inertial_forces", inertial_forces)
        vehicle.logger.log("vehicle_yaw_rate", yaw_rate)
        vehicle.logger.log("vehicle_x_dot", vehicle.x_dot)
        vehicle.logger.log("vehicle_y_dot", vehicle.y_dot)
        [vehicle.logger.log(name, val) for name, val in vehicle.state.items()]

        return np.array([*summation_forces, *summation_moments])