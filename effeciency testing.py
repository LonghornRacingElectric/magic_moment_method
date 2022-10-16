import pandas as pd
import math
EffMap = pd.read_csv('analysis/Eff228.csv')

def power_output(map, torque, w, angular_velocity = True):
    """

    :param map: csv map for torque vs rpm
    :param torque: torque input
    :param w: angular velocity input
    :param angular_velocity: set to false if input for w is in units of RPM instead of angular velocity
    :return: power output given torque and angular velocity input
    """
    torqueprox = round(torque)
    if angular_velocity:
        rpmprox = round(w*(60/(2*math.pi)))
    else:
        rpmprox = round(w)
        w = rpmprox*(2*math.pi)/60
    efficiency = map.iat[torqueprox,rpmprox]
    print(efficiency)
    return efficiency * torque * w

print(power_output(EffMap, 110, 3000, False))
