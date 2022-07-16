import math
from math import sin, cos

import numpy as np
import engine
from scipy.optimize import fsolve as josie_solver

"""
Coordinate Systems:
    IMF = SAE z-down wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html
    but with z up, y left^^^
"""

class Suspension():
    """ Handles suspension vehicle forces """
    def __init__(self, params, logger:engine.Logger):
        self.logger = logger
        self.params = params
        self.tires = engine.Tires(params)


    def get_loads(self, vehicle_velocity:np.array, yaw_rate:float, steered_angle:float, roll:float, pitch:float, ride_height:float):
        forces, moments = np.array([0, 0, 0]), np.array([0, 0, 0])
        # tire saturation & lifting
        saturated, normal_forces = [], []
        
        # tire output forces as a function of inclination angle, slip angle, and normal force
        for tire_name, tire in self.tires.items():
            
            ### ~~~ Normal Force Calculation ~~~ ###
            normal_force = self.get_tire_normal_load(tire_name, tire, ride_height, pitch, roll)
            
            ### ~~~ Slip Angle Calculation ~~~ ###
            tire_velocity = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]),tire.position) # in IMF
            slip_angle = math.atan2(tire_velocity[1], tire_velocity[0]) + tire.steering_induced_slip(steered_angle) # f(steered angle, body slip, yaw rate)
            
            ### ~~~ Inclination Angle Calculation ~~~ ###
            inclination_angle = self.get_inclination_angle(tire_name, tire, steered_angle, roll, ride_height, pitch)
            
            ### ~~~ Tire Output Force Calculation ~~~ ###
            f, m = self.get_tire_output(tire_name, tire, normal_force, slip_angle, inclination_angle)
            forces = np.add(f, forces)  
            moments = np.add(m, moments)
            
            ### ~~~ Logging Useful Information ~~~ ###
            self.logger.log(tire_name + "_tire_inclination_angle", inclination_angle)
            self.logger.log(tire_name + "_tire_velocity", tire_velocity)
            self.logger.log(tire_name + "_tire_slip_angle", slip_angle)
            self.logger.log(tire_name + "_tire_vehicle_centric_forces", f)
            self.logger.log(tire_name + "_tire_vehicle_centric_moments", m)
            grip_force_loss, grip_percent_loss = tire.lateral_loss(normal_force, slip_angle, inclination_angle)
            self.logger.log(tire_name + "_tire_inclination_angle_force_loss", grip_force_loss)
            self.logger.log(tire_name + "_tire_inclination_angle_percent_loss", grip_percent_loss)
            
            ### ~~~ Saturation & Lifting Checks ~~~ ###
            normal_forces.append(normal_force)
            saturated.append(tire.is_saturated(normal_force, slip_angle, inclination_angle))

        self.log_saturation(saturated, normal_forces)
        return forces, moments

    def log_saturation(self, saturateds:list, normals:list):
        # see if point is saturated (i.e. all 4 tires slip angles are saturated)
        saturated = not False in [tire_saturated for tire_saturated in saturateds]
        two_tires_lifting = sum([1 if (tire_normal == 0) else 0 for tire_normal in normals]) > 1
        
        self.logger.log("dynamics_tires_saturated", saturated)
        self.logger.log("dynamics_two_tires_lifting", two_tires_lifting)
          
    def get_inclination_angle(self, tire_name:str, tire:engine.Tire, steered_angle:float,
                                roll:float, ride_height:float, pitch:float):
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
          
    def get_tire_output(self, tire_name:str, tire:engine.Tire, normal_force:float, slip_angle:float, inclination_angle:float):
        lateral_force = tire.lateral_pacejka(inclination_angle, normal_force, slip_angle)
        tire_centric_forces = np.array([0, lateral_force, normal_force])
        
        rotation_matrix = np.array([[cos(slip_angle), -sin(slip_angle), 0],
                            [sin(slip_angle), cos(slip_angle),0],
                            [0,0,1]])
        
        # Rotate tire output into intermediate frame
        vehicle_centric_forces = np.dot(rotation_matrix, tire_centric_forces) 
        vehicle_centric_moments = np.cross(vehicle_centric_forces, tire.position)
        
        self.logger.log(tire_name + "_tire_tire_centric_forces", tire_centric_forces)
        
        return vehicle_centric_forces, vehicle_centric_moments
          
    def get_tire_normal_load(self, tire_name:str, tire:engine.Tire, ride_height:float, pitch:float, roll:float):
        heave = self.params.ride_height - ride_height
        
        specific_residual_func = lambda x: self.find_spring_displacements(x, tire_name, tire, heave, pitch, roll)
        tire_compression, wheel_displacement = josie_solver(specific_residual_func, [0.006, 0.001])

        normal_force = tire.tire_stiffness_func(tire_compression) * tire_compression
        
        self.logger.log(tire_name + "_tire_disp", tire_compression)
        self.logger.log(tire_name + "_tire_spring_disp", wheel_displacement)
        return normal_force

    def find_spring_displacements(self, x, tire_name:str, tire:engine.Tire, heave:float, pitch:float, roll:float):
        guess_tire_compression, guess_wheel_displacement = x

        wheelrate = tire.wheelrate_f(guess_wheel_displacement) 
        tire_stiffness = tire.tire_stiffness_func(guess_tire_compression)
        riderate = (wheelrate * tire_stiffness) / (wheelrate + tire_stiffness) 


        ### ~~~ Roll Contribution ~~~ ###
        # TODO: do about roll center
        # TODO: for the tire & spring conversion to roll stiffness, assuming L & R have same stiffness here; seems like an issue
        tire_contribution = tire_stiffness * tire.trackwidth ** 2 / 2 * (1 if tire.direction_left else -1)
        spring_contribution = wheelrate * tire.trackwidth ** 2 / 2 * (1 if tire.direction_left else -1)
        arb_contribution = tire.arb_stiffness
        # ARB in parallel with spring, tire in series with ARB and spring
        roll_stiffness = ((spring_contribution + arb_contribution) * tire_contribution /
                            ((spring_contribution + arb_contribution) + tire_contribution))
        f_roll = (roll_stiffness * roll) / tire.trackwidth
        x_wheel_roll = (spring_contribution / roll_stiffness) * f_roll / spring_contribution
        

        ### ~~~ Heave Contribution ~~~ ###
        f_heave = riderate * heave
        x_wheel_heave = f_heave / wheelrate


        ### ~~~ Pitch Contribution ~~~ ###
        # TODO: implement antisquat & antidive
        # TODO: do about pitch center
        pitch_heave = tire.position[0] * np.sin(pitch)
        f_pitch = riderate * pitch_heave
        x_wheel_pitch = f_pitch / wheelrate
        

        normal_force = f_roll + f_heave + f_pitch
        wheel_displacement = x_wheel_roll + x_wheel_heave + x_wheel_pitch
        tire_compression = normal_force / tire_stiffness

        self.logger.log(tire_name + "_tire_f_roll", f_roll)
        self.logger.log(tire_name + "_tire_f_heave", f_heave)
        self.logger.log(tire_name + "_tire_f_pitch", f_pitch)

        return guess_tire_compression - tire_compression, guess_wheel_displacement - wheel_displacement

    @property
    def avg_front_roll_stiffness(self):
        """ 
        Returns:
            float: average front roll stiffness in Nm / rad
        """
        return (abs(self.tires.front_right.roll_stiffness) + abs(self.tires.front_left.roll_stiffness)) / 2
        
    @property
    def avg_rear_roll_stiffness(self):
        """
        Returns:
            float: average rear roll stiffness in Nm / rad
        """
        return (abs(self.tires.rear_right.roll_stiffness) + abs(self.tires.rear_left.roll_stiffness)) / 2
        
    @property
    def rear_roll_stiffness_dist(self):
        """
        Returns:
            float: percent between 0 and 1 for the roll stiffness distribution rearwards
        """
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