import numpy as np
from vehicle import Vehicle
from scipy.optimize import fsolve as josie_solver
import pandas as pd
from copy import copy
from solver import DOF6_motion_residuals

vehicle = Vehicle()
vehicle.state.body_slip = 0.1
vehicle.state.steered_angle = 0
vehicle.state.x_dot = 10
vehicle.state.yaw_rate = 0

specific_residual_func = lambda x: DOF6_motion_residuals(x, vehicle)

# initial_guess (outputs) = ride_height, x_double_dot, y_double_dot, yaw_acceleration, roll, pitch
initial_guess = [ 7.60519137e-02, -1.46541521e+00,  1.46052723e+01, -1.07745954e+00,9.55283060e-04, -1.53428612e-02]

vehicle.state.body_slip = 0.1
vehicle.state.steered_angle = 0.1
vehicle.state.x_dot = 5
vehicle.state.yaw_rate = 0


print(vehicle.dynamics.get_loads(vehicle.state, *initial_guess[4:], initial_guess[0]))
#print(vehicle.suspension.get_loads(*vehicle.state.__dict__.values(),vehicle.y_dot, *initial_guess[4:], initial_guess[0]))

for name, tire in vehicle.dynamics.tires.__dict__.items():
    print(name, tire.outputs.slip_angle)
#print([tire.outputs.unsprung_displacement for tire in vehicle.dynamics.tires.__dict__.values()])
# (array([-3.98827401e+02,  3.97497090e+03, -8.65257226e-06]), array([-1009.64260944,  -101.30216156,  -115.50366099]))
# [0.00965852978462704, 0.0007391845350266405, 0.01050445680228461, 0.003736755993571225]
# front_left 0.00965852978462704
# front_right 0.0007391845350266405
# rear_left 0.01050445680228461
# rear_right 0.003736755993571225

# z_c
# 0.06719695037056206
# 0.08668391321759665
# 0.06648329014212408
# 0.08443584646577013

# print(vehicle.suspension.get_loads(*vehicle.state.__dict__.values(),vehicle.y_dot, *initial_guess[4:], initial_guess[0]))
# print([tire.unsprung_displacement for tire in vehicle.suspension.tires.__dict__.values()])
# (array([-631.50378073, 4327.98190397,   31.05441343]), array([-750.14923168,  -72.42154903, -481.13727164]))
# [0.007700549144159697, 0.0030192229621474106, 0.01050445680228461, 0.003736755993571225]

# front_left 0.007700549144159697
# front_right 0.0030192229621474106
# rear_left 0.01050445680228461
# rear_right 0.003736755993571225

# -0.005537753901996372
# -0.005537753901996372
# -0.006841485201793723
# -0.006841485201793723

            # z_c = ride_height + (tire.position[0]*sin(roll) + tire.position[1]*cos(roll)*sin(pitch)) \
            #     / (cos(roll) *cos(pitch))
            
            # # calculate unsprung displacements (from chasis displacement, stiffness); unsprung FBD
            # roll_stiffness = self.front_roll_stiffness if tire.steerable else self.rear_roll_stiffness
            # roll_stiffness *= 1 if tire.direction_left else -1
            # wheelrate_stiffness = self.front_wheelrate_stiffness if tire.steerable else self.rear_wheelrate_stiffness

            # unsprung_deformation_static = static_forces[name]/tire.stiffness
            # unsprung_height = tire.unloaded_radius + unsprung_deformation_static
            # static_chassis_height = self.ride_height #static_forces[name]/wheelrate_stiffness + unsprung_height # this is your STATIC CHASSIS CORNER HEIGHT FELLAS

            # # TODO: Change roll_stiffness in terms of differences of (chassis corner - unsprung displacements)
            # tire.unsprung_displacement = (roll_stiffness * roll + wheelrate_stiffness * (static_chassis_height - z_c)) \
            #     / tire.stiffness - unsprung_deformation_static   