from math import sin, cos

import numpy as np
import engine
from scipy.optimize import fsolve
import vehicle_params

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
        self.__tires = engine.Tires(params)


    def get_loads(self, vehicle_velocity:np.array, yaw_rate:float, steered_angle:float, roll:float, pitch:float, heave:float, slip_ratios:list):
        veh_forces, veh_moments, wheel_speeds, wheel_torques = np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([]), np.array([])
        
        # tire output forces as a function of inclination angle, slip angle, and normal force
        for tire_name, tire in self.__tires.items():
            
            ### ~~~ Normal Force Calculation ~~~ ###
            normal_force = self.__get_tire_normal_load(tire_name, tire, heave, pitch, roll)
            
            ### ~~~ Slip Angle Calculation ~~~ ###
            tire_IMF_velocity = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]), tire.position) # in IMF
            steering_toe_slip = tire.steering_induced_slip(steered_angle)
            slip_angle = np.arctan2(tire_IMF_velocity[1], tire_IMF_velocity[0]) +  steering_toe_slip# f(steered angle, body slip, yaw rate)
            
            ### ~~~ Inclination Angle Calculation ~~~ ###
            inclination_angle = self.__get_inclination_angle(tire_name, tire, steered_angle, roll, heave, pitch)
            
            ### ~~~ Tire Output Force Calculation ~~~ ###
            slip_ratio = tire.get_slip_ratio(slip_ratios)
            tire_forces, tire_moments, tire_torque = self.__get_tire_output(tire_name, tire, normal_force, slip_angle, inclination_angle, steering_toe_slip, slip_ratio)
            veh_forces = np.add(tire_forces, veh_forces)  
            veh_moments = np.add(tire_moments, veh_moments)
            wheel_torques = np.append(wheel_torques, [tire_torque])

            ### ~~~ Wheel Speeds from Slip Ratios and Tire Velocities ~~~ ###
            tire_pointing_unit_vector = np.array([np.cos(steering_toe_slip), np.sin(steering_toe_slip), 0])
            tire_pointing_velocity = tire_pointing_unit_vector.dot(tire_IMF_velocity)
            wheel_speed = (slip_ratio/100 + 1) * tire_pointing_velocity / tire.radius
            wheel_speeds = np.append(wheel_speeds, [wheel_speed])

            ### ~~~ Suspension Tube Force Calculation ~~~ ###
            tube_forces = [float(x) for x in self.get_tube_forces(tire, tire_forces * np.array([1, -tire.direction_left, 1]))]

            ### ~~~ Logging Useful Information ~~~ ###
            self.logger.log(tire_name + "_tire_inclination_angle", inclination_angle)
            self.logger.log(tire_name + "_tire_velocity", tire_IMF_velocity)
            self.logger.log(tire_name + "_tire_slip_angle", slip_angle)
            self.logger.log(tire_name + "_tire_torque", tire_torque)
            self.logger.log(tire_name + "_tire_vehicle_centric_forces", tire_forces)
            self.logger.log(tire_name + "_tire_vehicle_centric_moments", tire_moments)
            grip_force_loss, grip_percent_loss = tire.lateral_loss(normal_force, slip_angle, inclination_angle)
            self.logger.log(tire_name + "_tire_inclination_angle_force_loss", grip_force_loss)
            self.logger.log(tire_name + "_tire_inclination_angle_percent_loss", grip_percent_loss)
            self.logger.log(tire_name + "_tire_is_saturated", tire.is_saturated(normal_force, slip_angle, inclination_angle))
            self.logger.log(tire_name + "_FUCA_force", tube_forces[0])
            self.logger.log(tire_name + "_FLCA_force", tube_forces[1])
            self.logger.log(tire_name + "_RUCA_force", tube_forces[2])
            self.logger.log(tire_name + "_RLCA_force", tube_forces[3])
            self.logger.log(tire_name + "_pullrod_force", tube_forces[4])
            self.logger.log(tire_name + "_toe_link_force", tube_forces[5])

        return veh_forces, veh_moments, wheel_speeds, wheel_torques

          
    def __get_inclination_angle(self, tire_name:str, tire:engine.Tire, steered_angle:float,
                                roll:float, heave:float, pitch:float):
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
          
    def __get_tire_output(self, tire_name:str, tire:engine.Tire, normal_force:float, slip_angle:float,
                        inclination_angle:float, steering_slip:float, slip_ratio:list):

        tire_centric_forces = tire.comstock(slip_ratio, slip_angle, normal_force, inclination_angle)
        tire_torque = tire_centric_forces[0] * tire.radius
        
        rotation_matrix = np.array([[cos(steering_slip), -sin(steering_slip), 0],
                            [sin(steering_slip), cos(steering_slip),0],
                            [0,0,1]])
        
        # Rotate tire output into intermediate frame
        vehicle_centric_forces = np.dot(rotation_matrix, tire_centric_forces) 
        vehicle_centric_moments = np.cross(vehicle_centric_forces, tire.position)

        self.logger.log(tire_name + "_tire_torque", tire_torque)
        self.logger.log(tire_name + "_tire_tire_centric_forces", tire_centric_forces)

        return vehicle_centric_forces, vehicle_centric_moments, tire_torque


    def __get_tire_normal_load(self, tire_name:str, tire:engine.Tire, heave:float, pitch:float, roll:float):
        """ Gets tires normal force given vehicle displacements

        Args:
            tire_name (str): tire location
            tire (engine.Tire): tire object
            heave (float): vertical displacement - meters
            pitch (float): forward/backwards twist - radians
            roll (float): left/right twist - radians

        Returns:
            float: tire normal force - Newtons
        """        
        specific_residual_func = lambda x: self.__find_spring_displacements(x, tire_name, tire, heave, pitch, roll)
        tire_compression, wheel_displacement = fsolve(specific_residual_func, [0.006, 0.001])
        tire_compression = 0 if tire_compression < 0 else tire_compression
        normal_force = tire.tire_stiffness_func(tire_compression) * tire_compression
        normal_force = 0 if normal_force < 0 else normal_force
        
        self.logger.log(tire_name + "_tire_disp", tire_compression)
        self.logger.log(tire_name + "_tire_spring_disp", wheel_displacement)
        return normal_force


    def __find_spring_displacements(self, x:list, tire_name:str, tire:engine.Tire, heave:float, pitch:float, roll:float):
        """ Residual function for wheel displacement and tire compression to find tire normal load.

        Args:
            x (list): tire compression
            tire_name (str): tire location
            tire (engine.Tire): tire object
            heave (float): vertical displacement - meters
            pitch (float): forward/backwards twist - radians
            roll (float): left/right twist - radians

        Returns:
            list: residuals for wheel displacement and tire compression
        """
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
        roll_stiffness = - ((spring_contribution + arb_contribution) * tire_contribution /
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


    def get_tube_forces(self, tire, tire_forces):
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
    def avg_front_roll_stiffness(self):
        """ 
        Returns:
            float: average front roll stiffness in Nm / rad
        """
        return (abs(self.__tires.front_right.roll_stiffness) + abs(self.__tires.front_left.roll_stiffness)) / 2
        
    @property
    def avg_rear_roll_stiffness(self):
        """
        Returns:
            float: average rear roll stiffness in Nm / rad
        """
        return (abs(self.__tires.rear_right.roll_stiffness) + abs(self.__tires.rear_left.roll_stiffness)) / 2
        
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