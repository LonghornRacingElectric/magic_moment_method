import numpy as np
import pandas as pd
import itertools
import engine
import vehicle_params
import multiprocessing
import time
from tqdm import tqdm
import matplotlib.pyplot as plt
from time import perf_counter

solver = engine.Solver(vehicle_params.EasyDriver())

peak_slip_angle = 18 * np.pi / 180 # rad
refinement = 5

# s_dot_sweep = [12] # velocity sweep in path tangential direction (total velocity)
# body_slip_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
# steered_angle_sweep = np.linspace(-peak_slip_angle, peak_slip_angle, refinement)
# torque_request 1
# is_left_bias = np.array([True, False])

#     def __init__(self, body_slip, steered_angle, s_dot, torque_request = 0.0, is_left_bias = True):



x = solver.solve(engine.State(0 * np.pi / 180, 0 * np.pi/180, 15, .95, True))

tires = ["front_left","front_right", "rear_left", "rear_right"]

# for tire in tires:
#     for i in [0, 1, 2]:
#         print(x[tire + "_tire_tire_centric_forces_" + str(i)])
#         print(x[tire + "_tire_vehicle_centric_forces_" + str(i)])
fig, axs = plt.subplots(2,2,figsize=(16, 6), dpi=80)
plt.title("0 Steered Angle, -1 Degree Body Slip, 10% Accel Request, 12 m/s")
for index, tire in enumerate(tires):
    axs_i = axs[int(index/2)][index%2]
    slip_ratios = np.linspace(-1, 1, 100)
    slip_angle = x[tire + "_tire_slip_angle"]
    slip_ratio = x["wheel_speed_" + str(index + 1)]
    normal_force = x[tire + "_tire_tire_centric_forces_2"]
    long_force = x[tire + "_tire_tire_centric_forces_0"]
    lat_force = x[tire + "_tire_tire_centric_forces_1"]
    output = [solver.vehicle.suspension._Suspension__tires.rear_left.comstock(slip, -slip_angle, normal_force, 0) for slip in slip_ratios] # 
    #output2 = [solver.vehicle.suspension._Suspension__tires.front_left.comstock(10, slip/100, normal_force, 0) for slip in slip_ratios] # 
    axs_i.plot(slip_ratios, output)
    axs_i.plot(slip_ratios, [solver.vehicle.suspension._Suspension__tires.rear_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios])
    axs_i.plot(slip_ratios, [solver.vehicle.suspension._Suspension__tires.rear_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios])
    axs_i.scatter(slip_ratio, long_force)
    axs_i.scatter(slip_angle, lat_force)
    #axs_i.plot(slip_ratios, output2)
    axs_i.grid()
    # plt.title("SA = -7 degrees, SR = -10 -> 10, Front Tire, FZ = 500 N")
    axs_i.legend(["FX_com", "FY_com", "actual_FZ", "pure_long", "pure_lateral", "actual_SR", "actual_SA"])
    axs_i.set_title(tire)
plt.show()

#print(x["tire_torques_0"], x["tire_torques_1"], x["tire_torques_2"], x["tire_torques_3"])
print(x["wheel_speed_1"], x["wheel_speed_2"], x["wheel_speed_3"], x["wheel_speed_4"])




x = engine.Vehicle(vehicle_params.EasyDriver())

# slip_ratios = np.linspace(-.15, .15, 100)
# fz = 500
# output = [x.suspension._Suspension__tires.front_left.comstock(slip, 0.05, fz, 0) for slip in slip_ratios] # 
# plt.plot(slip_ratios, output)
# plt.plot(slip_ratios, [x.suspension._Suspension__tires.front_left.longitudinal_pacejka(fz, slip) for slip in slip_ratios])
# plt.plot(slip_ratios, [x.suspension._Suspension__tires.front_left.lateral_pacejka(0, fz, slip) for slip in slip_ratios])
# plt.grid()
# plt.title("SA = -7 degrees, SR = -10 -> 10, Front Tire, FZ = 500 N")
# plt.legend(["FX_com", "FY_com", "FZ_com", "pure_long", "pure_lateral"])
# plt.show()
# print(x.suspension._Suspension__tires.front_left.comstock(0.05, -5 * np.pi / 180, 500, 0))
# print(x.suspension._Suspension__tires.front_left.lateral_pacejka(0, fz, 0.05))



x = engine.Vehicle(vehicle_params.EasyDriver())

# slip_ratios = np.linspace(-.15, .15, 100)
# fz = 500
# output = [x.suspension._Suspension__tires.front_left.comstock(.05, slip, fz, 0) for slip in slip_ratios] # 
# plt.plot(slip_ratios, output)
# plt.plot(slip_ratios, [x.suspension._Suspension__tires.front_left.longitudinal_pacejka(fz, slip) for slip in slip_ratios])
# plt.plot(slip_ratios, [x.suspension._Suspension__tires.front_left.lateral_pacejka(0, fz, slip) for slip in slip_ratios])
# plt.grid()
# plt.title("SA = -7 degrees, SR = -10 -> 10, Front Tire, FZ = 500 N")
# plt.legend(["FX_com", "FY_com", "FZ_com", "pure_long", "pure_lateral"])
# plt.show()
# print(x.suspension._Suspension__tires.front_left.comstock(-0.001, 0.05, 500, 0))
#print(x.suspension._Suspension__tires.front_left.longitudinal_pacejka(500, 0.05))
# print(x.suspension._Suspension__tires.front_left.lateral_pacejka(0,500, 0.05))