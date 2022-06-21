import math
from math import sin, cos
from helpers.better_namespace import BetterNamespace
import numpy as np
from engine.front_tire import FrontTire
from engine.rear_tire import RearTire

"""
Coordinate Systems:
    SAE z-down wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html
    but with z up, y left^^^
    
Units: 
    Length: meters
    Mass: kg
"""

class Suspension():  
    def __init__(self, params):
        self.params = params
        
        self.outputs = BetterNamespace()
        # These two are metrics for throwing out data point as invalid
        # Since they couldn't exist in the real world
        self.outputs.tires_saturated = None
        self.outputs.two_tires_lifting = None
        
        self.outputs.total_inclination_angle_percent_loss = None
        self.outputs.total_inclination_angle_force_loss = None
        
        self.tires = BetterNamespace()
        self.tires.front_left = FrontTire(self.params, True)
        self.tires.front_right = FrontTire(self.params, False)
        self.tires.rear_left = RearTire(self.params, True)
        self.tires.rear_right = RearTire(self.params, False)

    def get_loads(self, vehicle_velocity, yaw_rate, steered_angle, roll, pitch, ride_height):
        # calculate unsprung intermediate (dependent) states
        self.set_unsprung_displacements(roll, pitch, ride_height)
        self.set_unsprung_slip_angles(vehicle_velocity, yaw_rate, steered_angle)
        self.set_unsprung_inclination_angles(roll, steered_angle)

        # get unsprung forces
        forces = np.array([0, 0, -self.params.mass * self.params.gravity])
        moments = np.array([0, 0, 0])

        # tire forces (inclination angle, slip angles, normal forces)
        for tire in self.tires.values():
            f, m = tire.get_loads()
            forces = np.add(f, forces)  
            moments = np.add(m, moments)

        self.log_overall_inclination_angle_loss()
        self.log_saturation()
        return forces, moments

    def log_saturation(self):
        # see if point is saturated (i.e. all 4 tires slip angles are saturated)
        self.outputs.tires_saturated = not False in [tire.is_saturated for tire in self.tires.values()]
        self.outputs.two_tires_lifting = sum([1 if tire.lifting else 0 for tire in self.tires.values()]) > 1

    def log_overall_inclination_angle_loss(self):
        loss = sum([abs(tire.outputs.inclination_angle_force_loss) for tire in self.tires.values()])
        force = sum([abs(tire.outputs.tire_centric_forces[1]) for tire in self.tires.values()])
        self.outputs.total_inclination_angle_force_loss = loss
        self.outputs.total_inclination_angle_percent_loss = loss / (force + loss)
            
    def set_unsprung_inclination_angles(self, roll, steered_angle):
        for tire in self.tires.values():
            disp = tire.wheel_displacement

            # TODO: what the fuck does the following mean lol
            # Tire swing length is the distance from the contact patch to (y, z) = (0, ride height).
            # Approximation: the angular displacement is the angle swept by this line as the tire displaces vertically
            # tire_swing_length = np.sqrt(self.params.ride_height ** 2 + np.abs(tire.position[1]) ** 2)
            # angular_displacement = np.sign(disp) * np.abs(np.arcsin(disp * np.abs(tire.position[1]) / (tire_swing_length \
            #                         * np.sqrt(disp ** 2 + tire_swing_length ** 2 - 2 * disp * self.params.ride_height))))
            #camber_gain_inclination = - tire.camber_gain * angular_displacement

            # Steering inclination: change in inclination angle due to steering
            # TODO: include heave/pitch induced camber gain
            
            tire.outputs.steering_inclination = tire.steered_inclination_angle_gain(steered_angle)
            
            roll_induced = roll - (roll * tire.camber_gain)

            tire.outputs.inclination_angle = roll_induced + tire.outputs.steering_inclination + tire.static_camber

    # slip angles (steered angle, body slip, yaw rate) and calculate forces/moments# STATIC TOE GOES HERE
    def set_unsprung_slip_angles(self, vehicle_velocity, yaw_rate, steered_angle):
        for tire in self.tires.values():
            # calculate tire velocities in IMF
            tire_velocity = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]),tire.position)
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
        
    @property
    def avg_front_roll_stiffness(self): # Nm / rad
        return (abs(self.tires.front_right.roll_stiffness) + abs(self.tires.front_left.roll_stiffness)) / 2
        
    @property
    def avg_rear_roll_stiffness(self): # Nm / rad
        return (abs(self.tires.rear_right.roll_stiffness) + abs(self.tires.rear_left.roll_stiffness)) / 2
        
    @property
    def rear_roll_stiffness_dist(self): # % rear/total 0->1 # TODO: does this need to account for roll height axis?
        return self.avg_rear_roll_stiffness / (self.avg_front_roll_stiffness + self.avg_rear_roll_stiffness)
    
    @property
    def roll_stiffness(self): # Nm / rad
        return self.avg_rear_roll_stiffness + self.avg_front_roll_stiffness

    @property
    def cg_weighted_roll_center(self): # m
        return (self.params.front_roll_center_height * (1 - self.params.cg_bias)
                + self.params.rear_roll_center_height * self.params.cg_bias) / 2
    
    @property
    def roll_stiffness_per_g(self): # rad/g # NOTE: roll calculation from M&M 18.4 (pg 682)
        cg_to_roll_axis = self.params.cg_total_position[2] - self.cg_weighted_roll_center
        kinetic_moment = self.params.mass_sprung * self.params.gravity * cg_to_roll_axis
        return - kinetic_moment / (self.roll_stiffness - kinetic_moment)