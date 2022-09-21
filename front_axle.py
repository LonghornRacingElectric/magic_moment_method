import numpy as np
from scipy.optimize import fsolve
from copy import copy
import warnings
import types


def redneck_pacejka(Fz, wheel_angular_velocity, tire_contact_patch_velocity):
    # TODO: solve for CP velocity using MMM states (dummy input for now)
    # TODO: Solve for loaded tire radius (using static value for now)
    r_effective = 0.4064
    slip_ratio = (wheel_angular_velocity * r_effective)/tire_contact_patch_velocity - 1
    return (Fz * slip_ratio * 10)*r_effective

# Params
b_bearing = 0.1
axle_MOI = 0.1

def ptn_residuals(derived_state_guess:list):
    # STATES: Assign manually here
    wheel_angular_velocity = 0.5

    # DERIVED_STATES:
    wheel_rot_accel = derived_state_guess[0]
    brake_request = derived_state_guess[1]


    # Intermediate calcs
    tire_Fz = np.array([30]) # placeholder
    wheel_torque = redneck_pacejka(tire_Fz, wheel_angular_velocity, 20)
    brake_torque = brake_request_to_torque(brake_request)

    residuals = np.zeros(2)

    residuals[0] = -axle_MOI*wheel_rot_accel - brake_torque[0] - b_bearing*wheel_angular_velocity + wheel_torque

    return residuals

def brake_request_to_torque(brake_request): # Brake request value from 0 to 1
    # Params
    max_pedal_force = 100
    pedal_ratio = 3
    master_cylinder_area = 0.2
    brake_bias_ratio = 0.6 # Percent of front
    rotor_radius = [0.2, 0.3]
    pad_area = [0.2, 0.3]
    rotor_pad_friction_coefficient = [0.8, 0.9]

    # Calcs
    pedal_force = (max_pedal_force*brake_request)*pedal_ratio
    master_pressure = [(pedal_force/master_cylinder_area)*brake_bias_ratio, (pedal_force/master_cylinder_area)*(1-brake_bias_ratio)]

    front_torque = master_pressure[0]*pad_area[0]*rotor_pad_friction_coefficient[0]*rotor_radius[0]
    rear_torque = master_pressure[1]*pad_area[1]*rotor_pad_friction_coefficient[1]*rotor_radius[1]

    return np.array([front_torque, rear_torque])


if __name__ == '__main__':
    initial_guess = [1, 0.6] # wheel_rot_accel, brake_request

    results = fsolve(ptn_residuals, initial_guess, full_output=True)

    print("Converged in ",  results[1]["nfev"], "iterations")
    print(results[0])
