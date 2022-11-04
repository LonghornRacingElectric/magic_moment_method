import numpy as np
from ..state_solver.state import State
from ..state_solver.logger import Logger
from ..state_solver.suspension import Suspension
from ..state_solver.aerodynamics import Aerodynamics
from ..state_solver.drivetrain import Drivetrain

"""
Coordinate Systems:
    IMF = SAE z-down wheel/tire centered coordinates EXCEPT with z up, y left
    https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html
"""

class Vehicle:
    def __init__(self, params, state:State = None):
        """_summary_

        Args:
            params (vehicle_params._vehicle_setup_): static & initial vehicle parameters
            state (State, optional): independent prescribed vehicle states. Defaults to None.
        """
        self.state = state
        self.logger = Logger()
        self.params = params        
        self.suspension = Suspension(self.params, self.logger)
        self.aero = Aerodynamics(self.params, self.logger)
        self.drivetrain = Drivetrain(self.params, self.logger)
    

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

    def get_loads(self, roll:float, pitch:float, heave:float, yaw_rate:float, slip_ratios:list):
        # gravity load
        gravity = np.array([0, 0, -self.params.mass * self.params.gravity])
        
        # Define aero loads
        aero_forces, aero_moments = self.aero.get_loads(self.state.IMF_vel[0], self.state.body_slip, pitch, roll,
                               heave)
        
        # Define tire loads (dynamics handles vehicle weight transfer through tire normals)
        tire_forces, tire_moments, wheel_speeds, tire_torques = self.suspension.get_loads(self.state.IMF_vel, yaw_rate,
                                                            self.state.steered_angle, roll, pitch, heave, slip_ratios)

        return aero_forces + tire_forces + gravity, aero_moments + tire_moments, wheel_speeds, tire_torques