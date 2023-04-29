import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("..")
import magic_moment_method.vehicle_params as vehicle_params
import magic_moment_method.state_solver as state_solver

### THIS IS USED FOR TESTING ONE DATA POINT

def main_long(result):
    tires = ["front_left","front_right", "rear_left", "rear_right"]
    slip_ratios = np.linspace(-1, 1, 100)
    _, axs = plt.subplots(2,2,figsize=(16, 6), dpi=80)
    plt.title("0 Steered Angle, -1 Degree Body Slip, 10% Accel Request, 12 m/s")
    for index, tire in enumerate(tires):
        axs_i = axs[int(index/2)][index%2]
        slip_angle = result[tire + "_tire_slip_angle"]
        slip_ratio = result[tire + "_tire_slip_ratio"]
        normal_force = result[tire + "_tire_tire_centric_forces_2"]
        long_force = result[tire + "_tire_tire_centric_forces_0"]
        if "front" in tire:
            comstock_output = [x.vehicle.suspension._Suspension__tires.front_left.get_comstock_forces(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
            long_paj = [x.vehicle.suspension._Suspension__tires.front_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios]
        else:
            comstock_output = [x.vehicle.suspension._Suspension__tires.rear_left.get_comstock_forces(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
            long_paj = [x.vehicle.suspension._Suspension__tires.rear_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios]
        axs_i.plot(slip_ratios, long_paj)
        axs_i.scatter(slip_ratio, long_force)
        axs_i.grid()
        axs_i.legend(["long_pacejka", "Actual data point"])
        axs_i.set_title(tire)
        axs_i.set_ylabel("Force (N)")
        axs_i.set_ylabel("Normalized Slip Ratio (%)")
    plt.show()

def main_lat(result):
    tires = ["front_left","front_right", "rear_left", "rear_right"]
    slip_ratios = np.linspace(-1, 1, 100)
    _, axs = plt.subplots(2,2,figsize=(16, 6), dpi=80)
    plt.title("0 Steered Angle, -1 Degree Body Slip, 10% Accel Request, 12 m/s")
    lat_force_total = 0
    for index, tire in enumerate(tires):
        axs_i = axs[int(index/2)][index%2]
        slip_angle = result[tire + "_tire_slip_angle"]
        normal_force = result[tire + "_tire_tire_centric_forces_2"]
        lat_force = result[tire + "_tire_tire_centric_forces_1"]
        if "front" in tire:
            comstock_output = [x.vehicle.suspension._Suspension__tires.front_left.get_comstock_forces(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
            lat_paj = [x.vehicle.suspension._Suspension__tires.front_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios]
        else:
            comstock_output = [x.vehicle.suspension._Suspension__tires.rear_left.get_comstock_forces(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
            lat_paj = [x.vehicle.suspension._Suspension__tires.rear_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios]
        axs_i.plot(slip_ratios, lat_paj)
        axs_i.scatter(slip_angle, lat_force)
        axs_i.grid()
        axs_i.legend(["lat_pacejka", "Actual data point"])
        axs_i.set_title(tire)
        axs_i.set_ylabel("Force (N)")
        axs_i.set_ylabel("Normalized Slip Angle (%)")
        print(f"{tire}: {int(normal_force)} N")
        lat_force_total += lat_force
    plt.show()

if __name__ == "__main__":
    x = state_solver.Solver(vehicle_params.Concept2023(motor_directory="vehicle_params/Eff228.csv"))
    result = x.solve(state_solver.State(0 * np.pi / 180, 0 * np.pi/180, 15, -0.57, False)) # body_slip, steered_angle, s_dot, torque_request, is_left_bias
    main_long(result)
    main_lat(result)