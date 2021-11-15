import math
from math import sin, cos
from better_namespace import BetterNamespace
import numpy as np
from tire import FrontTire, RearTire

"""
Coordinate Systems:
    SAE z-down wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html
    but with z up, y left^^^
    
Units: 
    Length: meters
    Mass: kg
"""

class Dynamics():
    def __init__(self, params):
        self.params = params
        self.outputs = BetterNamespace()

        self.tires = BetterNamespace()
        front_left_loc = [self.params.wheelbase * self.params.cg_bias, self.params.front_track/2, 0]
        front_right_loc = [self.params.wheelbase * self.params.cg_bias, -self.params.front_track/2, 0]
        rear_left_loc = [-self.params.wheelbase * (1-self.params.cg_bias), self.params.rear_track/2, 0]
        rear_right_loc = [-self.params.wheelbase * (1-self.params.cg_bias), -self.params.rear_track/2, 0]

        self.tires.front_left = FrontTire(self.params, front_left_loc, True)
        self.tires.front_right = FrontTire(self.params, front_right_loc, False)
        self.tires.rear_left = RearTire(self.params, rear_left_loc, True)
        self.tires.rear_right = RearTire(self.params, rear_right_loc, False)

    # see if point is saturated (i.e. all 4 tires slip angles are saturated)
    @property
    def tires_saturated(self):
        return not False in [tire.is_saturated for tire in self.tires.values()]

    def get_loads(self, vehicle_velocity, yaw_rate, steered_angle, roll, pitch, ride_height):
        # calculate unsprung states
        self.set_unsprung_displacements(roll, pitch, ride_height)
        self.set_unsprung_slip_angles(vehicle_velocity, yaw_rate, steered_angle)
        self.set_unsprung_inclination_angles(steered_angle)

        forces = np.array([0, 0, -self.params.mass * self.params.gravity])
        moments = np.array([0, 0, 0])

        # tire forces (inclination angle, slip angles, normal forces) # TODO slip ratio
        for tire in self.tires.values():
            f, m = tire.get_loads()
            forces = np.add(f, forces)  
            moments = np.add(m, moments)

        return forces, moments

    def set_unsprung_inclination_angles(self, steered_angle):
        for tire in self.tires.values():
            disp = tire.wheel_displacement

            l_static = np.sqrt(self.params.ride_height ** 2 + (tire.trackwidth / 2) ** 2)
            ang_disp = np.sign(disp) * np.abs(np.arcsin(disp * tire.trackwidth / (2 * l_static \
                                    * np.sqrt(disp ** 2 + l_static ** 2 - 2 * disp * self.params.ride_height))))
            cgain_inc = - tire.camber_gain * ang_disp

            tire.outputs.steering_inc = tire.steered_inclination_angle_gain(steered_angle) if type(tire) is FrontTire else 0
            tire.outputs.inclination_angle = cgain_inc + tire.outputs.steering_inc + tire.static_camber

    # slip angles (steered angle, body slip, yaw rate) and calculate forces/moments# STATIC TOE GOES HERE
    def set_unsprung_slip_angles(self, vehicle_velocity, yaw_rate, steered_angle):
        for tire in self.tires.values():
            # calculate tire velocities in IMF
            tire_velocity = vehicle_velocity + np.cross(np.array([0, yaw_rate, 0]),tire.position)
            slip_angle = math.atan2(tire_velocity[1], tire_velocity[0]) + tire.steering_induced_slip(steered_angle)
            
            tire.outputs.velocity = tire_velocity
            tire.outputs.slip_angle = slip_angle

    def set_unsprung_displacements(self, roll, pitch, ride_height):
        for tire in self.tires.values():
            # corner displacement of chassis
            # TODO: note rotation is about CG instead of roll / pitch centers
            # TODO: verify equation - why is it normalized with roll and pitch on the bottom?
            z_c = ride_height + (tire.position[0]*sin(pitch) + tire.position[1]*cos(pitch)*sin(roll)) \
                / (cos(pitch) *cos(roll))
            
            # calculate unsprung displacements (from suspension displacement, stiffness); unsprung FBD
            tire.set_unsprung_displacement(z_c, roll)