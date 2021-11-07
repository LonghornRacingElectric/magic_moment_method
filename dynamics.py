import math
from math import sin, cos, tan, tanh
import types
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

def bool_sgn(bool_var):
    if bool_var:
        return 1
    else:
        return -1

class Dynamics():
    def __init__(self, params):
        # TODO: this seems kinda funky but fine
        self.params = params

        self.tires = types.SimpleNamespace()
        front_left_loc = [self.params.wheelbase * self.params.cg_bias, self.params.front_track/2, 0]
        front_right_loc = [self.params.wheelbase * self.params.cg_bias, -self.params.front_track/2, 0]
        rear_left_loc = [-self.params.wheelbase * (1-self.params.cg_bias), self.params.rear_track/2, 0]
        rear_right_loc = [-self.params.wheelbase * (1-self.params.cg_bias), -self.params.rear_track/2, 0]

        self.tires.front_left = FrontTire(self.params, front_left_loc, True)
        self.tires.front_right = FrontTire(self.params, front_right_loc, False)
        self.tires.rear_left = RearTire(self.params, rear_left_loc, True)
        self.tires.rear_right = RearTire(self.params, rear_right_loc, False)

    def get_loads(self, state, roll, pitch, ride_height):
        # calculate unsprung states
        self.set_unsprung_displacements(roll, pitch, ride_height)
        self.set_unsprung_slip_angles(state)
        self.set_unsprung_inclination_angles(state)

        # TODO: is gravity weight correct to go here?
        forces = np.array([0, 0, -self.params.mass * self.params.gravity])
        moments = np.array([0, 0, 0])

        # tire forces (inclination angle, slip angles, normal forces) # TODO slip ratio
        for tire in self.tires.__dict__.values():
            f, m = tire.get_loads()
            forces = np.add(f, forces)  
            moments = np.add(m, moments)

        return forces, moments

    # function of steered angle and unsprung displacement
    def set_unsprung_inclination_angles(self, state):
        for tire in self.tires.__dict__.values():
            disp = tire.outputs.unsprung_displacement
            delta = state.steered_angle

            if type(tire) is FrontTire:
                track = self.params.front_track
                steer_inc = - tire.caster * delta + (1 / 2) * tire.KPI * np.sign(delta) * delta ** 2
            else:
                track = self.params.rear_track
                steer_inc = 0

            l_static = np.sqrt(self.params.ride_height ** 2 + (track / 2) ** 2)
            ang_disp = np.arcsin(disp * track / (2 * l_static \
                                    * np.sqrt(disp ** 2 + l_static ** 2 - 2 * disp * self.params.ride_height)))
            cgain_inc = - tire.camber_gain * ang_disp

            tire.outputs.inclination_angle = cgain_inc + steer_inc + tire.static_camber

    # slip angles (steered angle, body slip, yaw rate) and calculate forces/moments# STATIC TOE GOES HERE
    def set_unsprung_slip_angles(self, state):
        y_dot = state.x_dot * math.tan(state.body_slip)
        for tire in self.tires.__dict__.values():
            # calculate tire velocities in intermediate frame
            tire_velocity = np.array([state.x_dot, y_dot, 0]) + np.cross(np.array([0, state.yaw_rate, 0]),tire.position)
            tire.outputs.velocity = tire_velocity

            slip_angle = math.atan2(tire_velocity[1], tire_velocity[0])
            # TODO: Add in proper steering geometry later into FrontTire object
            slip_angle += tire.steering_induced_slip(state.steered_angle)
            tire.outputs.slip_angle = slip_angle

    def set_unsprung_displacements(self, roll, pitch, ride_height):
        for tire in self.tires.__dict__.values():
            # corner displacement of chassis
            # TODO: note rotation is about CG instead of roll / pitch centers
            z_c = ride_height + (tire.position[0]*sin(pitch) + tire.position[1]*cos(pitch)*sin(roll))
            #    / (cos(roll) *cos(pitch))  what's this normalization???
            
            # corner displacement of suspension
            # TODO: make sure this math makes sense
            # TODO: Change roll_stiffness in terms of differences of (chassis corner - unsprung displacements)
            z_roll = tire.roll_stiffness * roll
            z_wr = tire.wheelrate * (self.params.ride_height - z_c)
            z_total = z_roll + z_wr
            
            # calculate unsprung displacements (from suspension displacement, stiffness); unsprung FBD
            tire.set_unsprung_displacement(z_total)    