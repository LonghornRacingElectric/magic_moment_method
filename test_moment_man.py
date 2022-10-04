import numpy as np
import engine
import vehicle_params
import matplotlib.pyplot as plt


solver = engine.Solver(vehicle_params.EasyDriver())
result = solver.solve(engine.State(0 * np.pi / 180, 0 * np.pi/180, 15, -.9, True))

tires = ["front_left","front_right", "rear_left", "rear_right"]

fig, axs = plt.subplots(2,2,figsize=(16, 6), dpi=80)
plt.title("0 Steered Angle, -1 Degree Body Slip, 10% Accel Request, 12 m/s")
for index, tire in enumerate(tires):
    axs_i = axs[int(index/2)][index%2]
    slip_ratios = np.linspace(-1, 1, 100)
    slip_angle = result[tire + "_tire_slip_angle"]
    slip_ratio = result[tire + "_slip_ratio"]
    normal_force = result[tire + "_tire_tire_centric_forces_2"]
    long_force = result[tire + "_tire_tire_centric_forces_0"]
    lat_force = result[tire + "_tire_tire_centric_forces_1"]
    if "front" in tire:
        comstock_output = [solver.vehicle.suspension._Suspension__tires.front_left.comstock(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
        long_paj = [solver.vehicle.suspension._Suspension__tires.front_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios]
        lat_paj = [solver.vehicle.suspension._Suspension__tires.front_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios]
    else:
        comstock_output = [solver.vehicle.suspension._Suspension__tires.rear_left.comstock(slip, -slip_angle, normal_force, 0) for slip in slip_ratios]
        long_paj = [solver.vehicle.suspension._Suspension__tires.rear_left.longitudinal_pacejka(normal_force, slip) for slip in slip_ratios]
        lat_paj = [solver.vehicle.suspension._Suspension__tires.rear_left.lateral_pacejka(0, normal_force, slip) for slip in slip_ratios]
    axs_i.plot(slip_ratios, comstock_output)
    axs_i.plot(slip_ratios, long_paj)
    axs_i.plot(slip_ratios, lat_paj)
    axs_i.scatter(slip_ratio, long_force)
    axs_i.scatter(slip_angle, lat_force)
    axs_i.grid()
    axs_i.legend(["FX_com", "FY_com", "actual_FZ", "pure_long", "pure_lateral", "actual_SR", "actual_SA"])
    axs_i.set_title(tire)
plt.show()