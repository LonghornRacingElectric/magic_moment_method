import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append("..")
import magic_moment_method.vehicle_params as vehicle_params
import magic_moment_method.state_solver as state_solver


x = state_solver.Solver(vehicle_params.EasyDriver())
result = x.solve(state_solver.State(0 * np.pi / 180, 0 * np.pi/180, 15, -0.6, False)) # body_slip, steered_angle, s_dot, torque_request, is_left_bias
tires = ["front_left","front_right", "rear_left", "rear_right"]
slip_ratios = np.linspace(-1, 1, 100)
fig, axs = plt.subplots(2,2,figsize=(16, 6), dpi=80)
plt.title("0 Steered Angle, -1 Degree Body Slip, 10% Accel Request, 12 m/s")
for index, tire in enumerate(tires):
    axs_i = axs[int(index/2)][index%2]
    slip_angle = result[tire + "_tire_slip_angle"]
    slip_ratio = result[tire + "_slip_ratio"]
    normal_force = result[tire + "_tire_tire_centric_forces_2"]
    
    long_force = result[tire + "_tire_tire_centric_forces_0"]
    print(long_force)
    lat_force = result[tire + "_tire_tire_centric_forces_1"]
    if "front" in tire:
        comstock_output = [x.vehicle.suspension._Suspension__tires.front_left.comstock(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
        long_paj = [x.vehicle.suspension._Suspension__tires.front_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios]
        lat_paj = [x.vehicle.suspension._Suspension__tires.front_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios]
    else:
        comstock_output = [x.vehicle.suspension._Suspension__tires.rear_left.comstock(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
        long_paj = [x.vehicle.suspension._Suspension__tires.rear_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios]
        lat_paj = [x.vehicle.suspension._Suspension__tires.rear_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios]
    #axs_i.plot(slip_ratios, comstock_output)
    axs_i.plot(slip_ratios, long_paj)
    #axs_i.plot(slip_ratios, lat_paj)
    axs_i.scatter(slip_ratio, long_force)
    #axs_i.scatter(slip_angle, lat_force)
    axs_i.grid()
    #axs_i.legend(["FX_com", "FY_com", "actual_FZ", "pure_long", "pure_lateral", "actual_SR", "actual_SA"])
    axs_i.legend(["long_pacejka", "Actual data point"])
    axs_i.set_title(tire)
    axs_i.set_ylabel("Force (N)")
    axs_i.set_ylabel("Normalized Slip Ratio / Slip Angle (%)")
    # print(result[tire + "_tire_vehicle_centric_forces_0"])
    # print(result[tire + "_tire_steering_offset"])
    # print(slip_angle)
    #print(long_force)
print(result["vehicle_accelerations_NTB_0"])
print(result["vehicle_vehicle_forces_ntb_2"])
plt.show()


# slip_ratios = np.linspace(-1, 1, 15)
# normal_loads = np.array([500, 750, 1000])
# colors = np.array(["r", "g", "b"])
# for index, load in enumerate(normal_loads):
#     output = []
#     for slip_ratio in slip_ratios:
#         for slip_angle in slip_ratios:
#             comstock_output = x.vehicle.suspension._Suspension__tires.front_left.comstock(slip_ratio, slip_angle, load, 0)
#             output.append([comstock_output[0], comstock_output[1]])
#             plt.scatter(comstock_output[0], comstock_output[1], c= colors[index])
# plt.ylabel("Lateral Acceleration (m/s^2)")
# plt.xlabel("Long Acceleration (m/s^2)")
# plt.title("Long vs Lateral")
# plt.legend(normal_loads)
# plt.show()