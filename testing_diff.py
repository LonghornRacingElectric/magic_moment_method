import numpy as np
from scipy.optimize import fsolve
from copy import copy
import warnings
import types

types.SimpleNamespace()


def redneck_pacejka(Fz, wheel_angular_velocity, tire_contact_patch_velocity):
    # TODO: solve for CP velocity using MMM states (dummy input for now)
    # TODO: Solve for loaded tire radius (using static value for now)
    r_effective = 0.4064
    slip_ratio = (wheel_angular_velocity * r_effective)/tire_contact_patch_velocity - 1
    return (Fz * slip_ratio * 100)*r_effective

# Params
Jm = 0.1 # Motor inertia
Jd = 0.1 # Diff inertia
J3 =  0.1 # Left driveline inertia
J4 = 0.1 # Right driveline inertia
bm = 0.1 # Motor damping
bd = 0.1 # diff damping
b3 = 0.1 # wheel damping
chain_efficiency = 1
r_differential = 1 # radius of sprocket on diff side
r_motor = 3 # radius of sprocket on motor side
differential_efficiency = 1
differential_friction_loss_coefficient = 0

derived_state_results = types.SimpleNamespace()

def ptn_residuals(derived_state_guess:list):
    """
    Creates solver object
    States:
        w3, w4, motor_torque

    Derived States:
        wheel_angular_accel[4], motor_angular_velocity, diff_case_angular_velocity,
        motor_angular_accel, diff_case_angular_accel, chain_force, diff_input_torque, diff_output_torque
    Args:
        vehicle_parameters (vehicle_params._parameter_file_): specific static & initial vehicle parameters
    """
    # ASSIGN INDEPENDENT STATES MANUALLY HERE
    wheel_angular_velocity, motor_torque = np.array([0.5, 0.7]), 60

    # angular accels
    wheel_angular_accel = derived_state_guess
    diff_case_angular_accel = sum(wheel_angular_accel)/len(wheel_angular_accel)
    motor_angular_accel = diff_case_angular_accel * r_differential / r_motor

    # angular speeds
    diff_angular_velocity = sum(wheel_angular_velocity)/len(wheel_angular_velocity)
    motor_angular_velocity = diff_angular_velocity * r_differential / r_motor

    # torques
    tire_Fz = np.array([30, 23]) # placeholder
    wheel_torque = redneck_pacejka(tire_Fz, wheel_angular_velocity, np.array([15,20]))
    force_chain = (motor_torque - motor_angular_accel * Jm - motor_angular_velocity * bm) / r_motor
    torque_applied = force_chain * r_differential * differential_efficiency - diff_case_angular_accel * Jd - diff_angular_velocity * bd

    torque_out_diff = wheel_torque - wheel_angular_accel * J3 - wheel_angular_velocity * b3
    torque_traction_side = max(torque_out_diff)
    torque_slipping_side = min(torque_out_diff)


    diff_bias_ratio = 0.7 # TODO: incorporate ben's mapping from torque applied to diff bias (constant for now)

    residuals = np.zeros(2)
    residuals[0] = diff_bias_ratio * torque_applied - torque_traction_side
    residuals[1] = (1 - diff_bias_ratio) * torque_applied - torque_slipping_side

    derived_state_results.torque_applied = torque_applied

    return residuals


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    initial_guess = [0, 0] # wheel_rot_accel[2]

    results = fsolve(ptn_residuals, initial_guess, full_output=True)
    print(results[2])
    print(results)