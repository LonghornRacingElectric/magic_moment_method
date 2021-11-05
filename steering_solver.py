# Temporary file to test steering solving for MMM
import numpy as np
from scipy.optimize import fsolve as deeznutz_solve
import math
import matplotlib.pyplot as plt

def main():
    initial_guess = [math.radians(101.38), math.radians(176.39)]
    residual_func = lambda x: steering_residuals(x, 0)




    rack_displacements = np.linspace(-4.25, 4.25, num=50)
    steered_wheel_angle = np.empty(rack_displacements.size)

    for i in range(rack_displacements.size):
        residual_func = lambda x: steering_residuals(x, rack_displacements[i])
        theta1, theta2 = deeznutz_solve(residual_func, initial_guess)
        steered_wheel_angle[i] = math.degrees(theta1) - 90

    plt.plot(rack_displacements, steered_wheel_angle, label="right")
    plt.plot(-rack_displacements, -steered_wheel_angle, label="left")
    plt.title("Steering Geometry")
    plt.legend()
    plt.xlabel("Rack Displacement [in]")
    plt.ylabel("Steered Wheel Angle [deg]")
    plt.show()



def steering_residuals(thetas, rack_displacement):
    theta1, theta2 = thetas
    r1 = 5
    r2 = 11 #11.394
    L3 = 11.151
    L4 = 4

    r3 = np.sqrt(L4**2 + (-rack_displacement+L3)**2)
    meep = r1*np.cos(theta1)
    moop = r1*np.sin(theta1)
    beep = r2*np.cos(theta2)
    boop = r2*np.sin(theta2)
    residual1 = r1*np.cos(theta1)+r2*np.cos(theta2) + (L3-rack_displacement)
    residual2 = r1*np.sin(theta1)+r2*np.sin(theta2) - L4

    return np.array([residual1, residual2])


if __name__ == "__main__":
    main()