"""Configurations for all the files"""

import numpy as np
from typing import Sequence, Tuple
import ml_collections
from src.convex_mpc_controller.types import GaitType,ControllerMode,ConstraintTighteningMethod
from src.worlds import abstract_world,plane_world

def get_locomotion_controller_conf():
    
    """Configurations for the locomotion_controller.py file"""
    config = ml_collections.ConfigDict()
    
    # Maximum time in seconds to run the robot
    config.max_time:float = 25.0
   
    # Whether to show the GUI
    config.show_gui:bool = True
    
    # World to chose from
    config.world_class:abstract_world.AbstractWorld = \
    plane_world.PlaneWorld

    # Wait period for robot to react to DOWN action
    config.down_mode_sleep_time:float = 0.1

    # Wait period for robot to react to STAND action
    config.stand_mode_sleep_time:float = 0.001
    
    # Gait to chose from
    config.gait_type:GaitType = GaitType.TROT
    
    # The maximum number of constraint solver iterations.
    config.num_solver_iterations: int = 30
    
    # Every call to 'stepSimulation' the timeStep will proceed with timestep seconds
    config.timestep: float = 0.002
    
    # Desired height of body during locomotion
    config.desired_height:float = 0.32
    
    # Safety threshold for rotation of the robot
    config.up_vec_safe_threshold:float = 0.85
    
    # Safety threshold for the base height 
    config.base_height_safe_threshold:float = 0.18
    
    # Desired Controller mode to chose from
    config.desired_mode:ControllerMode = ControllerMode.WALK

    # Speed below which no ramping is applied
    config.ramp_long_speed_threshold_mps: float = 0.1

    config.ramp_lat_speed_threshold_mps: float = 0.1

    config.ramp_twisting_speed_threshold_radps: float = 0.1

    # When ramping to desired speed, do at this acceleration
    config.ramp_long_acceleration_mpss:float = 0.2

    config.ramp_lat_acceleration_mpss:float = 0.2

    config.ramp_twisting_acceleration_radpss:float = 0.2

    return config

def get_offset_gait_generator_conf():
    
    """Configurations specific to the offset_gait_generator.py file"""
    config = ml_collections.ConfigDict()
    
    # If desired leg phase is stance, actual leg phase is no contact
    # and normalized phase is higher than this constant, 
    # consider a lost contact
    config.lose_contact_phase_threshold:float = 0.1
    
    # If desired leg phase is swing, actual leg phase is in contact
    # and normalized phase is higher than this constant, 
    # consider a early touchdown
    config.early_touchdown_phase_threshold:float = 0.5
    
    return config

def get_com_velocity_estimator_conf():
    
    """Configurations specific to the com_velocity_estimator.py file"""
    config = ml_collections.ConfigDict()
    
    # Moving window size for low pass filtering for velocity estimation
    config.velocity_window_size:int = 60

    # Moving window size for low pass filtering for ground normal estimation
    config.ground_normal_window_size:int = 10
    
    # Avg noise in foot forces + this constant is the standing foot force calibration
    config.foot_force_cal_const_swing:float = 10.

    # Whether to update contact force threshold based on swing force history
    config.swing_leg_contact_force_cal:bool = False
    
    return config

def get_robot_conf():
    
    """Robot specific configurations"""
    config = ml_collections.ConfigDict()
    
    # Path for the Go1 Robot URDF
    config.urdf_path = "go1.urdf"
    
    # Names of the feet joints in the URDF
    config.foot_joint_names = ("FR_foot_fixed","FL_foot_fixed","RR_foot_fixed","RL_foot_fixed",)
    
    #####Configs for instantiating the MotorModels####
    
    # Hip Joint
    config.hip_init_pos:float = 0.0
    config.hip_min_pos:float = -0.802851455917
    config.hip_max_pos:float = 0.802851455917
    config.hip_min_vel:float = -16
    config.hip_max_vel:float = 16
    config.hip_min_torque:float = -23.7
    config.hip_max_torque:float = 23.7
    config.hip_kp:float = 100 
    config.hip_kd:float = 1 

    # Upper Joint
    config.upper_init_pos:float = 0.75
    config.upper_min_pos:float = -1.0471975512
    config.upper_max_pos:float = 4.18879020479
    config.upper_min_vel:float = -16
    config.upper_max_vel:float = 16
    config.upper_min_torque:float = -23.7
    config.upper_max_torque:float = 23.7 
    config.upper_kp:float = 100 
    config.upper_kd:float = 2 
    
    # Lower Joint
    config.lower_init_pos:float = -1.4
    config.lower_min_pos:float = -2.6965336943
    config.lower_max_pos:float = -0.916297857297
    config.lower_min_vel:float = -16
    config.lower_max_vel:float = 16
    config.lower_min_torque:float = -35.55
    config.lower_max_torque:float = 35.55 
    config.lower_kp:float = 100 
    config.lower_kd:float = 2 
    
    #################################################
    # Helper strings to form the motor model based on URDF
    config.hip_str:str = "_hip_joint"
    config.upper_str:str = "_thigh_joint"
    config.lower_str:str = "_calf_joint"
    
    #################################################
    # Hip positions in base frame
    config.swing_ref_pos:Tuple(Tuple(),Tuple(),Tuple(),Tuple()) = (
        (0.1835, -0.131, 0),
        (0.1835, 0.122, 0),
        (-0.195, -0.131, 0),
        (-0.195, 0.122, 0)
    )
    #################################################
    
    # Offset of the actual center of mass from the com frame
    config.com_offset:Sequence[float] = -np.array([0.011611, 0.004437, 0.000108])
    
    # Leg offset from trunk center
    config.hip_offsets:Sequence[float] = np.array([[0.1881, -0.04675, 0.], [0.1881, 0.04675, 0.],
                        [-0.1881, -0.04675, 0.], [-0.1881, 0.04675, 0.]]) + config.com_offset
    
    # Number of times to step the simulation
    config.action_repeat: int = 1
    
    # Every call to 'stepSimulation' the timeStep will proceed with timestep seconds
    config.timestep:float = get_locomotion_controller_conf().timestep
    
    # Initial positon on the rack
    config.init_rack_position: Tuple[float, float, float] = [0., 0., 1]
    
    # Initial position on the ground
    config.init_position: Tuple[float, float, float] = (0., 0., 0.32)
    
    # Is the robot hanging on a rack?
    config.on_rack: bool = False
    
    # Reset time / timestep no of iterations in sim to reset in sim
    config.reset_time_s: float = 3.

    # Motor angles when the robot is at rest i.e. lying on the ground
    config.rest_motor_angles:Sequence[float] = np.array([-0.1089, 1.2645, 2.7942] * 4)
    
    return config

def get_raibert_swing_leg_controller_conf():
    """Configurations specific to the raibert_swing_leg_controller.py file"""
    config = ml_collections.ConfigDict()
    
    # Whether to use Raibert Heuristic
    config.use_raibert_heuristic:bool = True
    
    # Position correction coefficients in Raibert's formula
    config.k_raibert = np.array([0.01, 0.01, 0.01])
    
    return config

def get_trot_conf():
    """Configurations specific to the trot gait type"""
    config = ml_collections.ConfigDict()
    
    ######### Desired Speed Settings ###########
    
    # Desired longitudinal speed
    config.desired_long_speed:float = 0.33
    
    # Desired lateral speed
    config.desired_lat_speed:float = 0.0
    
    # Desired angular speed
    config.desired_ang_speed:float = 0.0
    
    ######### Gait Generation Parameters ###########
    # Leg order is [FR, FL, RR, RL]
    
    # Frequency of gait cycle. Higher can allow dynamic gaits like flying_trot.
    stepping_frequency = 2.75
    
    # Phase Offsets: For a complete cycle of 2pi, 
    # these elements determine the offset in the phase of legs 2,3 and 4 wrt the FR leg. 
    # e.g. for trot and flying trot, the FR and RL legs are 
    # in phase (0) and the FL,RR are in phase with each other and out of phase wrt FR (pi)
    FL_phase_offset = np.pi; RR_phase_offset = np.pi; RL_phase_offset = 0.
    
    # Swing ratio: How much of one complete period should constitute
    # the swing cycle. So crawl~25% and trot gaits~50%.
    swing_ratio = 0.5
    
    # Instantiate the gait params
    config.gait_parameters:Sequence[float] =\
        [stepping_frequency,FL_phase_offset,RR_phase_offset,
        RL_phase_offset,swing_ratio]

    ######### Swing Leg Controller Landing Parameters ###########
    
    # At the end of swing, we leave a small clearance in meters to prevent unexpected foot collision
    config.foot_landing_clearance:float = -0.01
    
    # Apex of the swing height in meters when in swing motion 
    config.foot_height:float = 0.08
    
    # MPC planning horizon.
    config.planning_horizon_steps:float = 10 
    
    # MPC planning timestep
    config.planning_timestep:float = 0.025

    # MPC State and Control weights

    pos_x_wt = 0.0; pos_y_wt = 0.0; yaw_wt = 0.0; gravity_wt = 0.0

    ang_vel_x_wt = 0.2; ang_vel_y_wt = 0.2; ang_vel_z_wt = 1.0

    vel_x_wt = 20.0; vel_y_wt = 5.0

    pos_z_wt = 500

    vel_z_wt = 1e-6 

    roll_wt = 0.2; pitch_wt = 0.2
            
    fx_wt = 1e-6; fy_wt = 1e-6;fz_wt = 1e-6 

    # State tracking weights
    config.mpc_weights = (roll_wt, pitch_wt, yaw_wt,\
        pos_x_wt, pos_y_wt, pos_z_wt,\
        ang_vel_x_wt, ang_vel_y_wt, ang_vel_z_wt,\
        vel_x_wt, vel_y_wt, vel_z_wt,\
        gravity_wt)
    
    # Control penalties for all three dimensions for one foot
    config.mpc_control_weights = (fx_wt,fy_wt,fz_wt)

    return config

def get_flytrot_conf():
    """Configurations specific to the flytrot gait type"""
    config = ml_collections.ConfigDict()
    
    ######### Desired Speed Settings ###########
    
    # Desired longitudinal speed
    config.desired_long_speed:float = 1.0
    
    # Desired lateral speed
    config.desired_lat_speed:float = 0.
    
    # Desired angular speed
    config.desired_ang_speed:float = 0.
    
    ######### Gait Generation Parameters ###########
    # Leg order is [FR, FL, RR, RL]
    
    # Frequency of gait cycle. Higher can allow dynamic gaits like flying_trot.
    stepping_frequency = 2.5
    
    # Phase Offsets: For a complete cycle of 2pi, 
    # these elements determine the offset in the phase of legs 2,3 and 4 wrt the FR leg. 
    # e.g. for trot and flying trot, the FR and RL legs are 
    # in phase (0) and the FL,RR are in phase with each other and out of phase wrt FR (pi)
    FL_phase_offset = np.pi;RR_phase_offset = np.pi;RL_phase_offset = 0.
    
    # Swing ratio: How much of one complete period should constitute
    # the swing cycle. So crawl~25% and trot gaits~50%.
    swing_ratio = 0.65
    
    # Instantiate the gait params
    config.gait_parameters:Sequence[float] =\
        [stepping_frequency,FL_phase_offset,RR_phase_offset,\
        RL_phase_offset,swing_ratio]
    
    ######### Swing Leg Controller Landing Parameters ###########
    
    # At the end of swing, we leave a small clearance in meters to prevent unexpected foot collision
    config.foot_landing_clearance:float = -0.01
    
    # Apex of the swing height in meters when in swing motion 
    config.foot_height:float = 0.08
    
    # MPC planning horizon.
    config.planning_horizon_steps:float = 10
    
    # MPC planning timestep
    config.planning_timestep:float = 0.025

    # MPC State and Control weights
   
    pos_x_wt = 0.0; pos_y_wt = 0.0; yaw_wt = 0.0; gravity_wt = 0.0

    ang_vel_x_wt = 0.2; ang_vel_y_wt = 0.2; ang_vel_z_wt = 1.0

    vel_x_wt = 20.0; vel_y_wt = 5.0

    pos_z_wt = 500

    vel_z_wt = 1e-6 

    roll_wt = 0.2; pitch_wt = 0.2
    
    # lower values mean nmpc fails at lower weight 
    # in simulation, working with only two speeds
    control_multiplier = 1.
    if config.desired_long_speed == 0.5:
        print("control weight reduced")
        control_multiplier = 0.5 #TUNABLE
        
    fx_wt = 1e-6; fy_wt = 1e-6; fz_wt = 1e-6 

    # State tracking weights
    config.mpc_weights = (roll_wt, pitch_wt, yaw_wt,\
        pos_x_wt, pos_y_wt, pos_z_wt,\
        ang_vel_x_wt,ang_vel_y_wt, ang_vel_z_wt,\
        vel_x_wt, vel_y_wt, vel_z_wt,\
        gravity_wt)
    
    # Control penalties for all three dimensions for one foot
    config.mpc_control_weights = (fx_wt,fy_wt,fz_wt)

    return config

def get_torque_stance_leg_controller_conf():
    """
    Configurations specific to the torque_stance_leg_controller.py file
    Includes parameters for chance constraint tightening
    """
    config = ml_collections.ConfigDict()
    
    # MPC model mass, actual mass of the robot
    config.mpc_mass:float = 12.0

    # MPC model inertia. May need tuning
    config.mpc_inerta_x:float = 0.081 

    config.mpc_inerta_y:float = 0.268

    config.mpc_inerta_z:float = 0.302

    config.mpc_inertia:Sequence[float] = np.array((config.mpc_inerta_x, 0, 0, 0, config.mpc_inerta_y, 0, 0, 0, config.mpc_inerta_z)) 

    # MPC model surface friction
    config.mpc_surface_friction:float = 0.4
    
    # Number of legs
    config.num_legs:int = 4
    
    # Number of contact force dimensions
    config.force_dimension:int = 3 
    
    # Whether to zero out contact forces from MPC if disable contact
    config.disable_lose_contact:bool = True

    # Number of friction cone surfaces
    config.num_friction_cone_surfaces:int = 4
    
    # Number of robot parameters
    config.num_robot_parameters:int = 16

    # Number of robot states
    config.num_robot_states:int = 13

    # Number of robot dynamics states
    config.reduced_num_robot_states:int = 6

    # Chose between Nominal MPC, Heuristic MPC and Chance Constrained MPC
    # config.constraint_tightening_method:ConstraintTighteningMethod = ConstraintTighteningMethod.NONE
    config.constraint_tightening_method:ConstraintTighteningMethod = ConstraintTighteningMethod.CHANCECONSTRAINTS
    # config.constraint_tightening_method:ConstraintTighteningMethod = ConstraintTighteningMethod.HEURISTIC
    

    ###########################
    # Calculations for uncertainty in inverse mass Gaussian distribution
    # Variance is calculated off them
    ####################################################################
    # for reference nominal ixx:0.081, iyy: 0.268, izz: 0.302

    # Max inertial uncertainties
    max_uncertainty_inertia_x:float = 0.02
    max_uncertainty_inertia_y:float = 0.06
    max_uncertainty_inertia_z:float = 0.06

    # Max additional mass uncertainty 
    max_additional_mass:float = 15.0

    # Calculations for uncertainty in contact locations
    # Emperically we want to be able to climb up and down 
    # ~6cm of blocks.So derive the logic from that variance.
    # Aritrarily taking twice the value
    max_x_contact_location_uncertainty_m:float = 36. / 100.
    max_y_contact_location_uncertainty_m:float = 36. / 100.
    max_height_of_obstacle_m:float = 36. / 100.


    # Parameters for additive uncertainties
    max_angular_velocity_error_x:float = 0.5
    max_angular_velocity_error_y:float = 0.2
    max_angular_velocity_error_z:float = 0.01

    max_linear_velocity_error_x:float = 0.5
    max_linear_velocity_error_y:float = 0.2
    max_linear_velocity_error_z:float = 0.01


    ####################################################################
    # Calculation logic: We roughly consider the max uncertainties to be 3-σ.

    # Robot parameters
    var_Ixx = (max_uncertainty_inertia_x / 3.) ** 2
    var_Iyy = (max_uncertainty_inertia_y / 3.) ** 2
    var_Izz = (max_uncertainty_inertia_z / 3.) ** 2
    var_m   = (max_additional_mass / 3.) ** 2
    var_rx  = (max_x_contact_location_uncertainty_m / 3.) ** 2
    var_ry  = (max_y_contact_location_uncertainty_m / 3.) ** 2
    var_rz  = (max_height_of_obstacle_m / 3.) ** 2

    covariance_list = [var_Ixx, var_Iyy, var_Izz, var_m]
    contact_location_covariance_list = [var_rx,var_ry,var_rz] * config.num_legs
    covariance_list.extend(contact_location_covariance_list)

    config.Sigma_theta = np.diag((covariance_list))

    # Additive uncertainties
    var_omega_x = ( max_angular_velocity_error_x/ 3.) ** 2
    var_omega_y = ( max_angular_velocity_error_y/ 3.) ** 2
    var_omega_z = ( max_angular_velocity_error_z/ 3.) ** 2

    var_lin_vel_x = ( max_linear_velocity_error_x/ 3.) ** 2
    var_lin_vel_y = ( max_linear_velocity_error_y/ 3.) ** 2
    var_lin_vel_z = ( max_linear_velocity_error_z/ 3.) ** 2

    config.Sigma_w = np.diag( [var_omega_x,var_omega_y,var_omega_z,var_lin_vel_x,var_lin_vel_y,var_lin_vel_z] )

    ####################################################################

    # Constraint satisfaction probability threshold
    config.constraint_satisfaction_threshold:float = 0.95

    # State weights for DARE solution
    # Only robot dynamics states are considered
    ang_vel_x_wt:float = 1e-6 ; ang_vel_y_wt:float = 1e-6 ; ang_vel_z_wt:float = 1e-6 
    lin_vel_x_wt:float = 1e1 ; lin_vel_y_wt:float = 1e-6 ; lin_vel_z_wt:float = 1e-6

    config.dare_Q = np.diag([ang_vel_x_wt, ang_vel_y_wt, ang_vel_z_wt, lin_vel_x_wt, lin_vel_y_wt, lin_vel_z_wt])

    # Control weights for DARE solution. # Full control dimension of 12
    control_weight_fx:float = 1e-3; control_weight_fy:float = 1e-3; control_weight_fz:float = 1e-3
    config.dare_R = np.diag( [control_weight_fx,control_weight_fy,control_weight_fz] * config.num_legs )

    # Friction cone base matrix
    # Note since friction cone constraint tightening is done in <= form
    # The lb form used in the mpc is multiplied by -1
    # Additionally adding -f_z<=0 constraint to the mix
    config.friction_cone_base_matrix = np.array([
			[ 1.0, 0.0, -config.mpc_surface_friction],
			[-1.0, 0.0, -config.mpc_surface_friction],
			[0.0,  1.0, -config.mpc_surface_friction],
			[0.0, -1.0, -config.mpc_surface_friction],
            [0.0,  0.0, -1.0]
    ])

    # Scaling factor for minimum force in z-direction
    config.fz_min_scale:float = 0.0

    # Scaling factor for maximum force in z-direction
    config.fz_max_scale:float = 10.0

    ####### Computing heuristics for constraint tightening #######
    heuristic_max_mass:float = 10.0
    heuristic_max_accel:float = 0.2

    # Balance the additional mass equally across all 4 feet
    gravity_magnitude:float = 9.8
    heuristic_fz = (heuristic_max_mass * gravity_magnitude) / config.num_legs

    # Refer to paper for details. Roughly, max fx is max accel * max additional mass
    # The factors are scaled based on fx + mu*fz and fx- mu*fz
    factor_1 = config.mpc_surface_friction * heuristic_fz
    factor_2 = heuristic_max_mass * heuristic_max_accel

    heuristic_fc_1 = factor_1 - factor_2
    heuristic_fc_2 = factor_1 + factor_2

    # Heuristic constraint tightening for each of the four friction cone surfaces and unilateral contact forces
    config.heuristic_constraint_tightening_array = np.array([heuristic_fc_1,heuristic_fc_2,heuristic_fc_1,heuristic_fc_2,heuristic_fz])

    return config