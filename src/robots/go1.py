"""Derived Go1 sim class"""
from typing import Any

from src.robots.motors import MotorControlMode
from src.robots.motors import MotorGroup
from src.robots.motors import MotorModel
from src.robots.robot import Robot

# Configuration parameters for this file
from src.convex_mpc_controller import controller_configs
cfg = controller_configs.get_robot_conf()

class Go1(Robot):
    """Go1 Robot."""
    def __init__(self,pybullet_client: Any = None,motor_control_mode: MotorControlMode = MotorControlMode.POSITION,
                mpc_body_height=0.26):
        """Constructs an Go1 robot and resets it to the initial states.
        Initializes a tuple with a single MotorGroup containing 12 MotoroModels.
        Each MotorModel is by default configured for the parameters of the Go1."""
        
        motors = MotorGroup((
        MotorModel(
            name="FR"+cfg.hip_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.hip_init_pos,
            min_position=cfg.hip_min_pos,
            max_position=cfg.hip_max_pos,
            min_velocity=cfg.hip_min_vel,
            max_velocity=cfg.hip_max_vel,
            min_torque=cfg.hip_min_torque,
            max_torque=cfg.hip_max_torque,
            kp=cfg.hip_kp,
            kd=cfg.hip_kd,
        ),
        MotorModel(
            name="FR"+cfg.upper_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.upper_init_pos,
            min_position=cfg.upper_min_pos,
            max_position=cfg.upper_max_pos,
            min_velocity=cfg.upper_min_vel,
            max_velocity=cfg.upper_max_vel,
            min_torque=cfg.upper_min_torque,
            max_torque=cfg.upper_max_torque,
            kp=cfg.upper_kp,
            kd=cfg.upper_kd,
        ),
        MotorModel(
            name="FR"+cfg.lower_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.lower_init_pos,
            min_position=cfg.lower_min_pos,
            max_position=cfg.lower_max_pos,
            min_velocity=cfg.lower_min_vel,
            max_velocity=cfg.lower_max_vel,
            min_torque=cfg.lower_min_torque,
            max_torque=cfg.lower_max_torque,
            kp=cfg.lower_kp,
            kd=cfg.lower_kd,
        ),
        MotorModel(
            name="FL"+cfg.hip_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.hip_init_pos,
            min_position=cfg.hip_min_pos,
            max_position=cfg.hip_max_pos,
            min_velocity=cfg.hip_min_vel,
            max_velocity=cfg.hip_max_vel,
            min_torque=cfg.hip_min_torque,
            max_torque=cfg.hip_max_torque,
            kp=cfg.hip_kp,
            kd=cfg.hip_kd,
        ),
        MotorModel(
            name="FL"+cfg.upper_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.upper_init_pos,
            min_position=cfg.upper_min_pos,
            max_position=cfg.upper_max_pos,
            min_velocity=cfg.upper_min_vel,
            max_velocity=cfg.upper_max_vel,
            min_torque=cfg.upper_min_torque,
            max_torque=cfg.upper_max_torque,
            kp=cfg.upper_kp,
            kd=cfg.upper_kd,
        ),
        MotorModel(
            name="FL"+cfg.lower_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.lower_init_pos,
            min_position=cfg.lower_min_pos,
            max_position=cfg.lower_max_pos,
            min_velocity=cfg.lower_min_vel,
            max_velocity=cfg.lower_max_vel,
            min_torque=cfg.lower_min_torque,
            max_torque=cfg.lower_max_torque,
            kp=cfg.lower_kp,
            kd=cfg.lower_kd,
        ),
        MotorModel(
            name="RR"+cfg.hip_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.hip_init_pos,
            min_position=cfg.hip_min_pos,
            max_position=cfg.hip_max_pos,
            min_velocity=cfg.hip_min_vel,
            max_velocity=cfg.hip_max_vel,
            min_torque=cfg.hip_min_torque,
            max_torque=cfg.hip_max_torque,
            kp=cfg.hip_kp,
            kd=cfg.hip_kd,
        ),
        MotorModel(
            name="RR"+cfg.upper_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.upper_init_pos,
            min_position=cfg.upper_min_pos,
            max_position=cfg.upper_max_pos,
            min_velocity=cfg.upper_min_vel,
            max_velocity=cfg.upper_max_vel,
            min_torque=cfg.upper_min_torque,
            max_torque=cfg.upper_max_torque,
            kp=cfg.upper_kp,
            kd=cfg.upper_kd,
        ),
        MotorModel(
            name="RR"+cfg.lower_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.lower_init_pos,
            min_position=cfg.lower_min_pos,
            max_position=cfg.lower_max_pos,
            min_velocity=cfg.lower_min_vel,
            max_velocity=cfg.lower_max_vel,
            min_torque=cfg.lower_min_torque,
            max_torque=cfg.lower_max_torque,
            kp=cfg.lower_kp,
            kd=cfg.lower_kd,
        ),
        MotorModel(
            name="RL"+cfg.hip_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.hip_init_pos,
            min_position=cfg.hip_min_pos,
            max_position=cfg.hip_max_pos,
            min_velocity=cfg.hip_min_vel,
            max_velocity=cfg.hip_max_vel,
            min_torque=cfg.hip_min_torque,
            max_torque=cfg.hip_max_torque,
            kp=cfg.hip_kp,
            kd=cfg.hip_kd,
        ),
        MotorModel(
            name="RL"+cfg.upper_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.upper_init_pos,
            min_position=cfg.upper_min_pos,
            max_position=cfg.upper_max_pos,
            min_velocity=cfg.upper_min_vel,
            max_velocity=cfg.upper_max_vel,
            min_torque=cfg.upper_min_torque,
            max_torque=cfg.upper_max_torque,
            kp=cfg.upper_kp,
            kd=cfg.upper_kd,
        ),
        MotorModel(
            name="RL"+cfg.lower_str,
            motor_control_mode=motor_control_mode,
            init_position=cfg.lower_init_pos,
            min_position=cfg.lower_min_pos,
            max_position=cfg.lower_max_pos,
            min_velocity=cfg.lower_min_vel,
            max_velocity=cfg.lower_max_vel,
            min_torque=cfg.lower_min_torque,
            max_torque=cfg.lower_max_torque,
            kp=cfg.lower_kp,
            kd=cfg.lower_kd,
        ),
        ))
        
        self._mpc_body_height = mpc_body_height
        
        super().__init__(pybullet_client=pybullet_client,motors=motors)
    
    @property
    def swing_reference_positions(self):
        return cfg.swing_ref_pos
    
    @property
    def num_motors(self):
        return 12    
    
    @property
    def mpc_body_height(self):
        return self._mpc_body_height