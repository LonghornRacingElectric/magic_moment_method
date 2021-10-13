import math
from math import sin, cos, tan, tanh
import types
import numpy as np
from tire import Tire

"""
Coordinate Systems:
    SAE z-down wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html

Units: 
    Length: meters
    Mass: kg
"""


class Suspension():
    def __init__(self, params):
        self.params = params # to be passed on vehicle instantiation

        # Params (eventually move over to parameters file)
        self.front_track = 1.27
        self.rear_track = 1.17

        self.front_roll_stiffness = 500 * math.pi/180 #385 * math.pi/180  # N*m/rad
        self.rear_roll_stiffness = 385 * math.pi/180 # N*m/rad
        self.front_wheelrate_stiffness = (.574**2) * 400 / (.0254 * .224)
        self.rear_wheelrate_stiffness = (.747**2) * 450 / (.0254 * .224)

        self.front_toe = 0#4
        self.rear_toe = 0
        self.front_static_camber = 0
        self.rear_static_camber = 0

        # self.front_roll_center_height = -.75 * .0254
        # self.rear_roll_center_height = -.5 * .0254
        # self.pitch_center_x = -2.5 * 0.0254

        self.ride_height = 0.0762  # m

        rear_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        rear_coeff_Fy = [0.01131, -0.0314, 282.1, -650, -1490, 0.03926, -0.0003027, 0.9385, 5.777 * 10 ** -5, -0.06358,
                          -0.1176, 0.02715, 4.998, 5.5557 * 10 ** -5, 0.05059, 0.005199, 0.001232, 0.004013]
        rear_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        front_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        front_coeff_Fy = [1.311, -0.0003557, -2.586, 550.1, 1403, 0.00956, 3.087e-07,0.0004554, 0.0003098, 0.1428,
                         -0.1516, -0.1516, 0.304, 2.038e-05, 0.02862, 0.001671, -71.72, -281.9]
        front_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
 
        self.tires = types.SimpleNamespace()
        self.tires.front_left = Tire([front_coeff_Fx, front_coeff_Fy, front_coeff_Mz], [self.params.wheelbase * self.params.cg_bias, self.front_track/2, 0], 551 * 175, True, True, self.front_toe)
        self.tires.front_right = Tire([front_coeff_Fx, front_coeff_Fy, front_coeff_Mz], [self.params.wheelbase * self.params.cg_bias, -self.front_track/2, 0], 551 * 175, False, True, -self.front_toe)
        self.tires.rear_left = Tire([rear_coeff_Fx, rear_coeff_Fy, rear_coeff_Mz], [-self.params.wheelbase * (1-self.params.cg_bias), self.rear_track/2, 0], 669 * 175, True, False, self.rear_toe)
        self.tires.rear_right = Tire([rear_coeff_Fx, rear_coeff_Fy, rear_coeff_Mz], [-self.params.wheelbase * (1-self.params.cg_bias), -self.rear_track/2, 0], 669 * 175, False, False, -self.rear_toe)

    # TODO: Don't pass both bodyslip and y_dot
    def get_loads(self, body_slip, steered_angle, x_dot, yaw_rate, y_dot, roll, pitch, ride_height):
        # unsprung displacements (roll, pitch, ride height)
        self.get_unsprung_displacements(roll, pitch, ride_height)

        # normal forces (from unsprung displacements)
        forces = np.array([0, 0, -self.params.mass * self.params.gravity])
        moments = np.array([0, 0, 0])
        # # NOTE - THIS IS BEING DONE WITHIN TIRE

        # inclination angle (steered angle, unsprung displacements) # STATIC CAMBER GOES HERE

        # slip angles (steered angle, body slip, yaw rate) and calculate forces/moments# STATIC TOE GOES HERE
        # tire forces (inclination angle, slip angles, normal forces) # TODO slip ratio
        for name, tire in self.tires.__dict__.items():
            tire_velocity = np.array([x_dot, y_dot, 0]) + np.cross(np.array([0, yaw_rate, 0]),tire.position)
            slip_angle = math.atan2(tire_velocity[1], tire_velocity[0])

            slip_angle += steered_angle if tire.steerable else 0 # TODO: Add in proper steering geometry later

            tire_force = tire.get_force(slip_angle, 0, 0)  # Tire force in tire frame TODO: Current IA = 0 and SR = 0

            # Rotate the tire force into intermediate frame and add moments/forces to total
            # TODO: conversions utilities
            rotation_matrix = np.array([[cos(slip_angle),-sin(slip_angle),0],[sin(slip_angle),cos(slip_angle),0],[0,0,1]])
            tire_force_intermediate_frame = rotation_matrix.dot(tire_force)
            forces = np.add(forces, tire_force_intermediate_frame)
            moments = np.add(moments, np.cross(tire_force_intermediate_frame, tire.position))

        return forces, moments

    def get_unsprung_displacements(self, roll, pitch, ride_height):
        static_forces = self.static_weight_tire_forces

        for name, tire in self.tires.__dict__.items():
            # corner displacement of chasis
            # TODO: note rotation is about CG
            # TODO: CHECK EQUATION
            z_c = ride_height + (tire.position[0]*sin(roll) + tire.position[1]*cos(roll)*sin(pitch)) \
                / (cos(roll) *cos(pitch))
            
            # calculate unsprung displacements (from chasis displacement, stiffness); unsprung FBD
            roll_stiffness = self.front_roll_stiffness if tire.steerable else self.rear_roll_stiffness
            roll_stiffness *= 1 if tire.direction_left else -1
            wheelrate_stiffness = self.front_wheelrate_stiffness if tire.steerable else self.rear_wheelrate_stiffness

            unsprung_deformation_static = static_forces[name]/tire.stiffness
            unsprung_height = tire.unloaded_radius + unsprung_deformation_static
            static_chassis_height = self.ride_height #static_forces[name]/wheelrate_stiffness + unsprung_height # this is your STATIC CHASSIS CORNER HEIGHT FELLAS

            tire.unsprung_displacement = (roll_stiffness * roll + wheelrate_stiffness * (static_chassis_height - z_c)) \
                / tire.stiffness - unsprung_deformation_static      

    @property
    def static_weight_tire_forces(self):
        weight_front_static = -self.params.gravity * self.params.mass * (1 - self.params.cg_bias)
        weight_rear_static = -self.params.gravity * self.params.mass * self.params.cg_bias

        # Returning forces on tires in N
        return {"front_right":weight_front_static / 2, "front_left":weight_front_static / 2, "rear_right":weight_rear_static / 2, "rear_left":weight_rear_static / 2}