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
    def __init__(self, params, logger):
        self.logger = logger
        self.params = params
        
        self.tires = BetterNamespace()
        self.tires.front_left = FrontTire(self.params, True)
        self.tires.front_right = FrontTire(self.params, False)
        self.tires.rear_left = RearTire(self.params, True)
        self.tires.rear_right = RearTire(self.params, False)

    def get_loads(self, vehicle_velocity, yaw_rate, steered_angle, roll, pitch, ride_height):
        forces, moments = np.array([0, 0, 0]), np.array([0, 0, 0])
        # tire saturation & lifting
        saturated, normal_forces = [], []
        
        # tire output forces as a function of inclination angle, slip angle, and normal force
        for tire_name, tire in self.tires.items():
            
            ### ~~~ Normal Force Calculation ~~~ ###
            normal_force = self.get_tire_normal_load(tire_name, tire, ride_height, pitch, roll)
            
            ### ~~~ Slip Angle Calculation ~~~ ###
            slip_angle = self.get_tire_slip_angle(tire_name, tire, vehicle_velocity, yaw_rate, steered_angle)
            
            ### ~~~ Inclination Angle Calculation ~~~ ###
            inclination_angle = self.get_inclination_angle(tire_name, tire, steered_angle, roll, ride_height, pitch)
            
            ### ~~~ Tire Output Force Calculation ~~~ ###
            f, m = self.get_tire_output(tire_name, tire, normal_force, slip_angle, inclination_angle)
            forces = np.add(f, forces)  
            moments = np.add(m, moments)

            ### ~~~ Suspension Tube Force Calculation ~~~ ###
            tube_forces = [float(x) for x in self.get_tube_forces(forces)]

            ### ~~~ Logging Useful Information ~~~ ###
            self.logger.log(tire_name + "_tire_inclination_angle", inclination_angle)
            self.logger.log(tire_name + "_tire_slip_angle", slip_angle)
            self.logger.log(tire_name + "_tire_vehicle_centric_forces", f)
            self.logger.log(tire_name + "_tire_vehicle_centric_moments", m)
            grip_force_loss, grip_percent_loss = tire.lateral_loss(normal_force, slip_angle, inclination_angle)
            self.logger.log(tire_name + "_tire_inclination_angle_force_loss", grip_force_loss)
            self.logger.log(tire_name + "_tire_inclination_angle_percent_loss", grip_percent_loss)

            self.logger.log(tire_name + "FUCA_force", tube_forces[0])
            self.logger.log(tire_name + "FLCA_force", tube_forces[1])
            self.logger.log(tire_name + "RUCA_force", tube_forces[2])
            self.logger.log(tire_name + "RLCA_force", tube_forces[3])
            self.logger.log(tire_name + "pullrod_force", tube_forces[4])
            self.logger.log(tire_name + "toe_link_force", tube_forces[5])

            
            ### ~~~ Saturation & Lifting Checks ~~~ ###
            normal_forces.append(normal_force)
            saturated.append(tire.is_saturated(normal_force, slip_angle, inclination_angle))

        self.log_saturation(saturated, normal_forces)
        return forces, moments

    def log_saturation(self, saturateds, normals):
        # see if point is saturated (i.e. all 4 tires slip angles are saturated)
        saturated = not False in [tire_saturated for tire_saturated in saturateds]
        two_tires_lifting = sum([1 if (tire_normal == 0) else 0 for tire_normal in normals]) > 1
        
        self.logger.log("dynamics_tires_saturated", saturated)
        self.logger.log("dynamics_two_tires_lifting", two_tires_lifting)
          
    def get_inclination_angle(self, tire_name, tire, steered_angle, roll, ride_height, pitch):
        # TODO: what the fuck does the following mean lol
        #disp = self.wheel_displacement
        # Tire swing length is the distance from the contact patch to (y, z) = (0, ride height).
        # Approximation: the angular displacement is the angle swept by this line as the tire displaces vertically
        # tire_swing_length = np.sqrt(self.params.ride_height ** 2 + np.abs(tire.position[1]) ** 2)
        # angular_displacement = np.sign(disp) * np.abs(np.arcsin(disp * np.abs(tire.position[1]) / (tire_swing_length \
        #                         * np.sqrt(disp ** 2 + tire_swing_length ** 2 - 2 * disp * self.params.ride_height))))
        #camber_gain_inclination = - tire.camber_gain * angular_displacement
        
        steering_induced = tire.steered_inclination_angle_gain(steered_angle)
        roll_induced = tire.roll_inclination_angle_gain(roll)
        heave_induced = 0 # TODO
        pitch_induced = 0 # TODO   
        
        self.logger.log(tire_name + "_tire_steering_inclination", steering_induced)
        self.logger.log(tire_name + "_tire_roll_inclination", roll_induced)
        self.logger.log(tire_name + "_tire_heave_inclination", heave_induced)
        self.logger.log(tire_name + "_tire_pitch_inclination", pitch_induced)
        
        return tire.static_camber + steering_induced + roll_induced + heave_induced + pitch_induced
            
          
    def get_tire_slip_angle(self, tire_name, tire, vehicle_velocity, yaw_rate, steered_angle):
        # calculate tire velocities in IMF
        tire_velocity = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]),tire.position)
        # slip angles (steered angle, body slip, yaw rate)
        slip_angle = math.atan2(tire_velocity[1], tire_velocity[0]) + tire.steering_induced_slip(steered_angle)
        
        self.logger.log(tire_name + "_tire_velocity", tire_velocity)
    
        return slip_angle
          
    def get_tire_output(self, tire_name, tire, normal_force, slip_angle, inclination_angle):
        # note: don't allow normal force of 0 to produce tire forces
        if normal_force < 0:
            lateral_force = 0
            tire_centric_forces = np.array([0, 0, 0])
            vehicle_centric_forces = np.array([0, 0, 0])
            vehicle_centric_moments = np.zeros(3)
        else:
            lateral_force = tire.lateral_pacejka(inclination_angle, normal_force, slip_angle)
            tire_centric_forces = np.array([0, lateral_force, normal_force])
            
            rotation_matrix = np.array([[cos(slip_angle), -sin(slip_angle), 0],
                                [sin(slip_angle), cos(slip_angle),0],
                                [0,0,1]])
            
            # Rotate the tire force into intermediate frame
            vehicle_centric_forces = np.dot(rotation_matrix, tire_centric_forces) 
            vehicle_centric_moments = np.cross(vehicle_centric_forces, tire.position)
        
        self.logger.log(tire_name + "_tire_tire_centric_forces", tire_centric_forces)

        return vehicle_centric_forces, vehicle_centric_moments
          
    def get_tire_normal_load(self, tire_name, tire, ride_height, pitch, roll):
        # corner displacement of chassis
        # TODO: note rotation is about CG instead of roll / pitch centers
        # TODO: verify equation - why is it normalized with roll and pitch on the bottom?
        z_c = ride_height + (tire.position[0]*sin(pitch) + tire.position[1]*cos(pitch)*sin(roll)) \
            / (cos(pitch) *cos(roll))
        
        f_roll = tire.roll_force(roll)
        # TODO: don't use private tire parameters outside of tire class
        # calculate unsprung displacements (from suspension displacement, stiffness); unsprung FBD
        tire_compression = (f_roll + tire.wheelrate * z_c) / (tire.tire_springrate + tire.wheelrate)
        f_heave = tire.riderate * z_c
        normal_force = tire.tire_springrate * tire_compression 
        
        self.logger.log(tire_name + "_tire_f_roll", f_roll)
        self.logger.log(tire_name + "_tire_f_heave", f_heave)
        self.logger.log(tire_name + "_tire_z_c", z_c)
        
        return normal_force

    def get_tube_forces(self, tire_forces, tire):
        normal_vects = tire.tube_geometry[0]
        lever_arms = tire.tube_geometry[1]

        # Set Up Force and Moment Balance; Create unit vectors that forces and moments will be applied to
        arr_cross = np.cross(normal_vects.T, lever_arms.T).T
        arr_coeff = np.concatenate((normal_vects, arr_cross))

        # Solve
        b = np.concatenate((tire_forces, np.zeros(3)))
        tube_forces = np.linalg.solve(arr_coeff, b)

        return tube_forces
                   
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