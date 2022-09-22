import engine
import numpy as np
import vehicle_params

"""
Coordinate Systems:
    IMF = SAE z-down wheel/tire centered coordinates EXCEPT with z up, y left
    https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html
"""

class Vehicle:
    def __init__(self, params:vehicle_params.BaseVehicle, state:engine.State = None):
        """_summary_

        Args:
            params (vehicle_params._vehicle_setup_): static & initial vehicle parameters
            state (engine.State, optional): independent prescribed vehicle states. Defaults to None.
        """
        self.state = state
        self.logger = engine.Logger()
        self.params = params        
        self.suspension = engine.Suspension(self.params, self.logger)
        self.aero = engine.Aerodynamics(self.params, self.logger)
    

    # TODO: CoG movements due to roll / pitch / heave
    def get_kinetic_moments(self, linear_accelerations:np.array, angular_accelerations:np.array):
        """eq. 17-9 pg. 425 in Hibbler Engineering Mechanics Dynamics textbook

        Args:
            linear_accelerations (np.array): 3-D acceleration array in NTB directions
            angular_accelerations (np.array): 3-D angular acceleration array about NTB axes

        Returns:
            np.array: 3-D moment array about NTB axes
        """
        cg_relative_ntb = np.array([0, 0, self.params.cg_total_position[2]])
        m_a_term = np.cross(self.params.mass * linear_accelerations, cg_relative_ntb)
        I_alpha_term = np.dot(self.params.sprung_inertia, angular_accelerations)

        self.logger.log("vehicle_yaw_moment", I_alpha_term[2])

        return m_a_term + I_alpha_term
    

    def get_inertial_forces(self, linear_accelerations:np.array):       
        return self.params.mass * linear_accelerations


    # normal to path acceleration (lateral accel) passed in; yaw rate dependent state on this as described by below equations
    # NOTE: turn radius being added to outputs for logging
    def get_yaw_rate(self, lateral_accel:float):
        # no slip condition; yaw rate guaranteed by acceleration and velocity
        if lateral_accel == 0:
            turn_radius = 0
            yaw_rate = 0
        else:
            # v^2/r = a; w*r = v; w = v/r = v/(v^2/a); alpha = a/r
            turn_radius = self.state.s_dot ** 2 / lateral_accel
            yaw_rate = self.state.s_dot / turn_radius
            
        self.logger.log("vehicle_turn_radius", turn_radius)

        return yaw_rate


    @property
    def translational_velocities_IMF(self):
        """
        Returns:
            np.array([0:2]): velocity vector in vehicle coordinate frame (IMF)
        """
        return np.array([self.x_dot, self.y_dot, 0])
    

    def intermediate_frame_to_ntb_transform(self, intermediate_frame_vector:np.array):
        """ Transforms input vector using body slip

        Args:
            intermediate_frame_vector (np.array([0:2])): input vector in vehicle coordinate frame (IMF)

        Returns:
            np.array([0:2]): output vector in normal tangential coordinate frame (NTB)
        """
        transformation_matrix = np.linalg.inv(np.array([[np.cos(self.state.body_slip), -np.sin(self.state.body_slip), 0],
                                          [np.sin(self.state.body_slip), np.cos(self.state.body_slip), 0],
                                          [0, 0, 1]]))
        return np.dot(transformation_matrix, intermediate_frame_vector)       
    

    @property
    def x_dot(self):
        return self.state.s_dot * np.cos(self.state.body_slip)
    

    @property
    def y_dot(self):
        return self.state.s_dot * np.sin(self.state.body_slip)


    def get_loads(self, roll:float, pitch:float, heave:float, yaw_rate:float):
        # gravity load
        gravity = np.array([0, 0, -self.params.mass * self.params.gravity])
        
        # Define aero loads
        aero_forces, aero_moments = self.aero.get_loads(self.x_dot, self.state.body_slip, pitch, roll,
                               heave)
        
        # Define tire loads (dynamics handles vehicle weight transfer through tire normals)
        tire_forces, tire_moments, wheel_speeds, wheel_torques = self.suspension.get_loads(self.translational_velocities_IMF, yaw_rate,
                                                            self.state.steered_angle, roll, pitch, heave, self.state.slip_ratios)
                                                            
        return aero_forces + tire_forces + gravity, aero_moments + tire_moments, wheel_speeds, wheel_torques,

    def brake_request_to_torque(self, brake_request): # Brake request value from 0 to 1
        # Params
        max_pedal_force = 100
        pedal_ratio = 3
        master_cylinder_area = 0.2
        brake_bias_ratio = 0.6 # Percent of front
        rotor_radius = [0.2, 0.3]
        pad_area = [0.2, 0.3]
        rotor_pad_friction_coefficient = [0.8, 0.9]

        # Calcs
        pedal_force = (max_pedal_force*brake_request)*pedal_ratio
        master_pressure = [(pedal_force/master_cylinder_area)*brake_bias_ratio, (pedal_force/master_cylinder_area)*(1-brake_bias_ratio)]

        front_torque = master_pressure[0]*pad_area[0]*rotor_pad_friction_coefficient[0]*rotor_radius[0]
        rear_torque = master_pressure[1]*pad_area[1]*rotor_pad_friction_coefficient[1]*rotor_radius[1]

        return np.array([front_torque, rear_torque])