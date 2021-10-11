import math
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
    def __init__(self):

        self.param = types.SimpleNamespace()
        self.state = types.SimpleNamespace()
        self.env = types.SimpleNamespace()

        # Environment variables (temporary for outside of lapsim functionality)
        self.env.g = 9.81

        # Params (eventually move over to parameters file)
        self.param.front_track = 1.27
        self.param.rear_track = 1.17
        self.param.wheelbase = 1.55

        self.param.front_roll_stiffness = 385 * math.pi/180  # N*m/rad
        self.param.rear_roll_stiffness = 385 * math.pi/180 # N*m/rad
        self.param.pitch_stiffness = 0
        self.param.front_wheelrate_stiffness = (.574**2) * 400 / (.0254 * .224)
        self.param.rear_wheelrate_stiffness = (.747**2) * 450 / (.0254 * .224)

        self.param.front_toe = 4
        self.param.rear_toe = 0
        self.param.front_static_camber = 0
        self.param.rear_static_camber = 0

        self.param.mass_unsprung_front = 13.5
        self.param.unsprung_front_height = 0.0254 * 8
        self.param.mass_unsprung_rear = 13.2
        self.param.unsprung_rear_height = 0.0254 * 8

        self.param.mass_sprung = 245.46
        self.param.cg_sprung_position = np.array([1.525 * 0.60, 0, 0.0254 * 10])

        self.param.mass_total = self.param.mass_sprung + self.param.mass_unsprung_front + self.param.mass_unsprung_rear
        self.param.cg_total_bias = 0.6  # Position of the cg from front to rear, value from 0-1
        self.param.cg_total_position = np.array([self.param.cg_total_bias * self.param.wheelbase, 0, 0.0254 * 10])

        self.param.front_roll_center_height = -.75 * .0254
        self.param.rear_roll_center_height = -.5 * .0254
        self.param.pitch_center_x = -2.5 * 0.0254

        self.param.ride_height = 0.0762  # m

        rear_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        rear_coeff_Fy = [0.01131, -0.0314, 282.1, -650, -1490, 0.03926, -0.0003027, 0.9385, 5.777 * 10 ** -5, -0.06358,
                          -0.1176, 0.02715, 4.998, 5.5557 * 10 ** -5, 0.05059, 0.005199, 0.001232, 0.004013]
        rear_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        front_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        front_coeff_Fy = [1.311, -0.0003557, -2.586, 550.1, 1403, 0.00956, 3.087e-07,0.0004554, 0.0003098, 0.1428,
                         -0.1516, -0.1516, 0.304, 2.038e-05, 0.02862, 0.001671, -71.72, -281.9]
        front_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    
        front_tire_pacejka = [front_coeff_Fx, front_coeff_Fy, front_coeff_Mz]
        rear_tire_pacejka = [rear_coeff_Fx, rear_coeff_Fy, rear_coeff_Mz]
        # States
        self.state.tires = {"front_left": Tire(front_tire_pacejka), "front_right": Tire(front_tire_pacejka)
            , "rear_left": Tire(rear_tire_pacejka), "rear_right": Tire(rear_tire_pacejka)}

        self.state.bodyslip = 0
        # self.state.yaw = 0
        self.state.pitch = 0
        self.state.roll = 0

        self.state.steer_angle = 0

        self.state.position = np.array([0, 0])  # Planar position in the global frame, maybe should be RacecarState
        self.state.velocity = np.array([0, 0])

        self.state.accel = np.array([0, 0, 0])

        self.update_tires()

    # def steady_state_load_transfer(self, Ax, Ay, Az):
    #     longitudinal_reaction_moment = Ax * self.mass_total
    #     lateral_reaction_moment = Ay * self.mass_total
    #
    #     total_roll_stiffness = self.front_roll_stiffness + self.rear_roll_stiffness
    #
    #     self.state.pitch = longitudinal_reaction_moment/self.pitch_stiffness
    #     self.state.roll = lateral_reaction_moment/total_roll_stiffness
    #
    #
    #     self.state.front_left_tire.state.force[2] = (self.mass_total * (1 - self.cg_bias) / 2) \
    #                                                 - (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                 - (lateral_reaction_moment / self.front_track) / ( 2 / self.front_roll_stiffness/total_roll_stiffness)
    #     self.state.front_right_tire.state.force[2] = (self.mass_total * (1 - self.cg_bias) / 2) \
    #                                                 - (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                 + (lateral_reaction_moment / self.front_track) / ( 2 / self.front_roll_stiffness/total_roll_stiffness)
    #     self.state.rear_left_tire.state.force[2] = (self.mass_total * self.cg_bias / 2) \
    #                                                + (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                - (lateral_reaction_moment / self.front_track) / ( 2 / self.rear_roll_stiffness/total_roll_stiffness)
    #     self.state.rear_left_tire.state.force[2] = (self.mass_total * self.cg_bias / 2) \
    #                                                + (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                + (lateral_reaction_moment / self.front_track) / ( 2 / self.rear_roll_stiffness/total_roll_stiffness)
    
    def get_loads(self):
        forces = np.array([0, 0, 0])
        moments = np.array([0, 0, 0])
        return forces, moments

    def get_total_Fy(self):
        total_Fy = 0
        for tire in self.state.tires.values():
            total_Fy += tire.get_Fy()*math.cos(tire.state.slip_angle)

        return total_Fy
    def get_total_yaw_moment(self):
        yaw_moment = 0
        yaw_moment += (self.state.tires["front_left"].get_Fy())
        return
    def update_tires(self):
        # Add the body slip to the slip angle on each tire
        for tire in self.state.tires.values():
            tire.state.slip_angle = self.state.bodyslip


        # Update toe, steering to each tire (following standard toe sign, steered angle direction same as body slip)
        self.state.tires["front_left"].state.slip_angle += (-self.param.front_toe + self.state.steer_angle)
        self.state.tires["front_right"].state.slip_angle += (self.param.front_toe + self.state.steer_angle)
        self.state.tires["rear_left"].state.slip_angle += (-self.param.rear_toe)
        self.state.tires["rear_right"].state.slip_angle += self.param.rear_toe

        # Update camber
        # TODO: add camber gain
        self.state.tires["front_left"].state.camber = self.param.front_static_camber
        self.state.tires["front_right"].state.camber = self.param.front_static_camber
        self.state.tires["rear_left"].state.camber = self.param.rear_static_camber
        self.state.tires["rear_right"].state.camber = self.param.rear_static_camber

        # Update normal forces
        static_tire_weights = self.get_static_weight_tire_forces()
        [dW_lon, dW_lat_f, dW_lat_r] = self.get_static_weight_transfer()
        self.state.tires["front_left"].state.force[2] = static_tire_weights[0] + dW_lat_f + dW_lon
        self.state.tires["front_right"].state.force[2] = static_tire_weights[1] - dW_lat_f + dW_lon
        self.state.tires["rear_left"].state.force[2] = static_tire_weights[2] + dW_lat_r - dW_lon
        self.state.tires["rear_right"].state.force[2] = static_tire_weights[3] - dW_lat_r - dW_lon

        # Update lateral forces
        for tire in self.state.tires.values():
            tire.state.force[1] = tire.get_Fy()

    def get_static_weight_tire_forces(self):
        weight_front_static = -self.env.g * self.param.mass_total * (1 - self.param.cg_total_bias)
        weight_rear_static = -self.env.g * self.param.mass_total * self.param.cg_total_bias

        # Returning forces on tires in N
        return [weight_front_static / 2, weight_front_static / 2, weight_rear_static / 2, weight_rear_static / 2]

    def get_static_weight_transfer(self):
        # dW_lats are always 0 unless there is a nonzero lateral acceleration
        # dW_long is positive when accelerating

        a_lon = self.state.accel[0] / self.env.g
        a_lat = self.state.accel[1] / self.env.g

        ic_geometry = self.ic_geometry()

        # Longitudinal load transfer
        dW_lon = 0.5 * a_lon * self.param.mass_total * self.env.g * self.param.cg_total_position[2] / self.param.wheelbase

        # pitch = self.params.springs.p_GR[p_direction] * a_lon

        # Roll calculation - from M&M 18.4 (pg 682), requires roll stiffnesses to be in terms of radians for small angle
        # to hold
        weight_sprung = self.param.mass_sprung * self.env.g

        K_roll = weight_sprung * ic_geometry[2] / ((self.param.front_roll_stiffness + self.param.rear_roll_stiffness)
                                                   - (weight_sprung * ic_geometry[2]))  # rad/g

        roll = a_lat * K_roll * 180 / math.pi  # Roll in deg

        # Front roll load transfer
        K_R_f_prime = self.param.front_roll_stiffness - (self.param.wheelbase - self.param.cg_sprung_position[0]) \
                      * weight_sprung * ic_geometry[2] / self.param.wheelbase

        dw_spr_f = (a_lat * weight_sprung / self.param.front_track) * (ic_geometry[2] * K_R_f_prime /
                   (self.param.front_roll_stiffness + self.param.rear_roll_stiffness - weight_sprung * ic_geometry[2]))

        dw_geo_f = (a_lat * weight_sprung / self.param.front_track) * \
                   ((self.param.wheelbase - self.param.cg_sprung_position[0]) / self.param.wheelbase) * ic_geometry[0]

        weight_unsprung_front = self.param.mass_unsprung_front * self.env.g
        dw_uns_f = a_lat * weight_unsprung_front * (self.param.unsprung_front_height / self.param.front_track)

        # Rear roll load transfer
        K_R_r_prime = self.param.rear_roll_stiffness - self.param.cg_sprung_position[0] \
                      * weight_sprung * ic_geometry[2] / self.param.wheelbase

        dw_spr_r = (a_lat * weight_sprung / self.param.rear_track) * (ic_geometry[2] * K_R_r_prime /
                   (self.param.front_roll_stiffness + self.param.rear_roll_stiffness - weight_sprung * ic_geometry[2]))

        dw_geo_r = (a_lat * weight_sprung / self.param.rear_track) * \
                   self.param.cg_sprung_position[0] / self.param.wheelbase * ic_geometry[1]

        weight_unsprung_rear = self.param.mass_unsprung_rear * self.env.g
        dw_uns_r = a_lat * weight_unsprung_rear * (self.param.unsprung_rear_height / self.param.rear_track)

        dW_lat_f = dw_spr_f + dw_geo_f + dw_uns_f
        dW_lat_r = dw_spr_r + dw_geo_r + dw_uns_r

        return [dW_lon, dW_lat_f, dW_lat_r]

    def stiffness_arb(self):
        if self.param.arb.enabled_f:
            K_TB_f = self.param.arb.G_TB_f * math.pi / 32 * \
                     (self.param.arb.TB_f_OD ** 4 -
                      self.param.arb.TB_f_ID ** 4) / self.param.arb.TB_f_L

            self.K_ARB_f = K_TB_f * self.param.arb.IR_f ** 2 * (self.param.car.TK_f / self.param.arb.LA_f) ** 2 \
                           * math.pi / 180  # Nm/deg

        if self.param.arb.enabled_r:
            K_TB_r = self.param.arb.G_TB_r * math.pi / 32 * \
                     (self.param.arb.TB_r_OD ** 4 -
                      self.param.arb.TB_r_ID ** 4) / self.param.arb.TB_r_L

            self.K_ARB_r = K_TB_r * self.param.arb.IR_r ** 2 * \
                           (self.param.car.TK_r / self.param.arb.LA_r) ** 2 * math.pi / 180

    def stiffness_suspension(self):
        k_W_f = self.param.wheel.IR_f ** 2 * self.param.wheel.k_SP_f
        k_RIDE_f = k_W_f * self.param.front_tire.k_T / \
                   (k_W_f + self.param.front_tire.k_T)
        self.K_S_f = k_RIDE_f * \
                     (self.params.car.TK_f / 2) ** 2 * 2 * math.pi / 180  # N.m/deg

        k_W_r = self.params.wheel.IR_r ** 2 * self.params.wheel.k_SP_r
        k_RIDE_r = k_W_r * self.params.rear_tire.k_T / \
                   (k_W_r + self.params.rear_tire.k_T)
        self.K_S_r = k_RIDE_r * \
                     (self.params.car.TK_r / 2) ** 2 * 2 * math.pi / 180

    def ic_geometry(self):
        RA_theta = math.atan2((self.param.front_roll_center_height - self.param.rear_roll_center_height),
                              self.param.wheelbase)
        height_cg_to_roll_center = (self.param.cg_sprung_position[2] - self.param.front_roll_center_height) * math.cos(RA_theta) - \
                                   self.param.cg_sprung_position[2] * self.param.wheelbase * math.sin(RA_theta)

        return [self.param.front_roll_center_height, self.param.rear_roll_center_height, height_cg_to_roll_center,
                self.param.pitch_center_x]

    def find_body_slip_and_steer(self, bodyslip_sweep, steer_angle_sweep):
        possible_pairs = np.zeros((1, 3))
        for bodyslip in bodyslip_sweep:
            for steer_angle in steer_angle_sweep:
                self.state.bodyslip = bodyslip
                self.state.steer_angle = steer_angle
                self.update_tires()

                if self.state.accel[1]*self.param.mass_total * 0.95 <= self.get_total_Fy() \
                        <= self.state.accel[1] * self.param.mass_total * 1.05:
                    possible_pairs = np.vstack([possible_pairs,[bodyslip, steer_angle, self.get_total_Fy()]])
                    print(bodyslip, steer_angle)

        return possible_pairs
