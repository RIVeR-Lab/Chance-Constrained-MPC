"""
Main controller interface for Go1 quadrupedal locomotion. 
Configurations specific to each component are set up in the 
controller_configs.py file
"""

import numpy as np
import pybullet
from pybullet_utils import bullet_client
import time

import os
current_file_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(os.path.dirname(current_file_dir))
os.sys.path.insert(0,parent_dir)

# Module specific imports
from src.convex_mpc_controller import com_velocity_estimator
from src.convex_mpc_controller import offset_gait_generator
from src.convex_mpc_controller import raibert_swing_leg_controller
from src.convex_mpc_controller import torque_stance_leg_controller
from src.convex_mpc_controller.types import GaitType, ControllerMode
from src.robots import go1
from src.robots.motors import MotorCommand
from src.robots.motors import MotorControlMode
from src.worlds import plane_world

# Configuration parameters for this file
from src.convex_mpc_controller import controller_configs
cfg = controller_configs.get_locomotion_controller_conf()

class LocomotionController:
    """Generates the quadruped locomotion.
    
    The actual effect of this controller depends on the composition of each
    individual subcomponent.
    """

    def __init__(self):
        
        """Initialized the class"""
        self._show_gui = cfg.show_gui
        self._world_class = cfg.world_class
        self._setup_robot_and_controllers()
        self.reset_robot()
        self.reset_controllers()
        self._reset_time = self._clock()
        self._time_since_reset = 0
        self._mode = ControllerMode.DOWN
        self.set_controller_mode(cfg.desired_mode)
        self._unsafe_termination = False
        self.run()

    def run(self):
        """Main locomotion loop"""
        
        try:
            start_time = self.time_since_reset
            current_time = start_time

            while True and (current_time - start_time < cfg.max_time):
                
                # Update current time
                current_time = self.time_since_reset

                # Check robot safety
                if self.is_safe == False:
                    self._unsafe_termination = True
                    print("Robot fell terminating simulation...")
                    # break

                # Check if desired mode has changed
                self._handle_mode()

                # Update computations from individual components
                self.update()

                # Actions specific to each mode
                if self._mode == ControllerMode.DOWN:
                    time.sleep(cfg.down_mode_sleep_time)
                elif self._mode == ControllerMode.STAND:
                    action = self._get_stand_action()
                    self._robot.step(action)
                    time.sleep(cfg.stand_mode_sleep_time)
                elif self._mode == ControllerMode.WALK:
                    # The second output qp_sol can be used for plotting, logging etc
                    init_time = time.time()
                    action, _ = self.get_action()
                    self._robot.step(action)
                    while time.time() - init_time <= cfg.timestep:
                        pass
                else:
                    print("Running loop terminated, exiting...")
                    break
            
                # Camera Setup
                if self._show_gui:
                    self.pybullet_client.resetDebugVisualizerCamera(
                        cameraDistance=1.0,
                        cameraYaw=self._robot.base_orientation_rpy[2] / np.pi * 180,
                        cameraPitch=-10,
                        cameraTargetPosition=self._robot.base_position)
                    
        except Exception as e:
            print("Exception in locomotion controller: {}".format(e))

        finally:
            self.set_controller_mode(ControllerMode.TERMINATE)
            exit()

    def _handle_mode(self):
        if self._mode == self._desired_mode:
            return
        
        self._mode = self._desired_mode
        
        if self._desired_mode== ControllerMode.DOWN:
            print("Entering joint damping mode controller mode")
        elif self._desired_mode == ControllerMode.STAND:
            print("Standing up controller mode")
            self.reset_robot()
        else:
            print("Walking controller mode")
            self.reset_controllers()
            
    def set_controller_mode(self, mode:ControllerMode=ControllerMode.STAND):
        self._desired_mode = mode 
    
    @property
    def swing_leg_controller(self):
        return self._swing_controller
    
    @property
    def stance_leg_controller(self):
        return self._stance_controller
    
    @property
    def gait_generator(self):
        return self._gait_generator
    
    @property
    def state_estimator(self):
        return self._state_estimator
      
    @property
    def time_since_reset(self):
        return self._time_since_reset
        
    def reset_robot(self):
        self._robot.reset(hard_reset=False)
        if self._show_gui==True:
            self.pybullet_client.configureDebugVisualizer(self.pybullet_client.COV_ENABLE_RENDERING, 1)   
    
    def reset_controllers(self):
        # Resetting other components
        self._reset_time = self._clock()
        self._time_since_reset = 0
        self._gait_generator.reset()
        self._state_estimator.reset(self._time_since_reset)
        self._swing_controller.reset(self._time_since_reset)
        self._stance_controller.reset(self._time_since_reset)
        
    def update(self):
        self._time_since_reset = self._clock() - self._reset_time
        self._gait_generator.update()
        self._state_estimator.update(self._gait_generator.desired_leg_state)
        self._swing_controller.update(self._time_since_reset)
        future_contact_estimate = self._gait_generator.get_estimated_contact_states(
            self._gait_config.planning_horizon_steps, self._gait_config.planning_timestep)
        self._stance_controller.update(self._time_since_reset, future_contact_estimate)

    def get_action(self):
        """Returns the control ouputs (e.g. positions/torques) for all motors."""

        # Generate desired speeds
        self._ramp_speed()
        
        swing_action = self._swing_controller.get_action(self._desired_speed, self._desired_twisting_speed)
        stance_action, qp_sol = self._stance_controller.get_action(self._desired_speed, self._desired_twisting_speed)
        
        actions = []
        
        for joint_id in range(self._robot.num_motors):
            if joint_id in swing_action:
                actions.append(swing_action[joint_id])
            else:
                assert joint_id in stance_action
                actions.append(stance_action[joint_id])
        
        vectorized_action = MotorCommand(
        desired_position=[action.desired_position for action in actions],
        kp=[action.kp for action in actions],
        desired_velocity=[action.desired_velocity for action in actions],
        kd=[action.kd for action in actions],
        desired_extra_torque=[
            action.desired_extra_torque for action in actions
        ])

        return vectorized_action, dict(qp_sol=qp_sol)    

    def _setup_robot_and_controllers(self):
        
        """
        Create the pybullet simulation, instantiate world, robot class, state estimator, 
        gait generator, swing and stance leg controller
        """
        
        # Set up gait parameters
        if cfg.gait_type == GaitType.TROT:
            print("Gait Type: Trot")
            self._gait_config = controller_configs.get_trot_conf()
        elif cfg.gait_type == GaitType.FLYTROT:
            print("Gait Type: Flytrot")
            self._gait_config = controller_configs.get_flytrot_conf()         
        else:
            raise ValueError("Invalid Gait Type")

        # Construct pybullet instance
        if self._show_gui == True:
            # Enable the GUI
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            # Not activating the display
            p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
        
        # Disable rendering to speed up following calculations
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        # Enable searching for robot URDF 
        p.setAdditionalSearchPath('src/data')

        # Make a copy of the pybullet client
        self.pybullet_client = p

        # Makes visualization better
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Hide the entire GUI

        # The maximum number of constraint solver iterations
        p.setPhysicsEngineParameter(numSolverIterations=cfg.num_solver_iterations)

        # Amount the world simulates forward when stepsimulation is called.
        p.setTimeStep(cfg.timestep)

        # Gravity set in the Z direction which is facing up
        p.setGravity(0, 0, -9.8)

        # Disable cone friction and approximate using pyramid instead
        p.setPhysicsEngineParameter(enableConeFriction=0)

        # Create a world instance
        world = self._world_class(self.pybullet_client)

        # Build world
        world.build_world()
        
        # Construct robot class:
        self._robot = go1.Go1(pybullet_client=p, motor_control_mode=MotorControlMode.HYBRID,
                                  mpc_body_height=cfg.desired_height)
        
        if cfg.world_class == plane_world.PlaneWorld and cfg.gait_type == GaitType.TROT:
            # Attach wooden planks
            self._robot.place_wooden_planks()
        
        # Re-enable visualization
        if self._show_gui == True:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        
        # Stopwatch
        self._clock = lambda: self._robot.time_since_reset
        
        # Setup the desired speed variables. To prevent jerky motions, the desirred
        # speeds are slowly ramped up before passing onto the leg controllers
        self._desired_long_speed = 0.
        self._desired_lat_speed = 0.
        self._desired_twisting_speed = 0.

        self._last_ramp_time = self._clock()

        # Instantiate gait generator
        self._gait_generator = offset_gait_generator.OffsetGaitGenerator(self._robot,self._gait_config.gait_parameters)
    
        # Instantiate center of mass state estimator
        self._state_estimator = com_velocity_estimator.COMVelocityEstimator(self._robot,cfg.desired_height)
        
        # Instantiate swing leg controller
        self._swing_controller = raibert_swing_leg_controller.RaibertSwingLegController(
            self._robot,
            self._gait_generator,
            self._state_estimator,
            desired_speed=(self._gait_config.desired_long_speed, self._gait_config.desired_lat_speed),
            desired_twisting_speed=self._gait_config.desired_ang_speed,
            desired_height=cfg.desired_height,
            foot_landing_clearance=self._gait_config.foot_landing_clearance,
            foot_height=self._gait_config.foot_height)
        
        # Instantiate stance leg controller
        self._stance_controller = torque_stance_leg_controller.TorqueStanceLegController(
            self._robot,
            self._gait_generator,
            self._state_estimator,
            desired_speed=(self._gait_config.desired_long_speed, self._gait_config.desired_lat_speed),
            desired_twisting_speed=self._gait_config.desired_ang_speed,
            desired_body_height=cfg.desired_height,
            planning_horizon_steps=self._gait_config.planning_horizon_steps,
            planning_timestep=self._gait_config.planning_timestep,
            mpc_weights=self._gait_config.mpc_weights)
     
    def _get_stand_action(self):
        return MotorCommand(
            desired_position=self._robot.motor_group.init_positions,
            kp=self._robot.motor_group.kps,
            desired_velocity=0,
            kd=self._robot.motor_group.kds,
            desired_extra_torque=0)
    
    def _ramp_speed(self):
        """Based on the eventual desired speed and 
        the max acceleration rate, ramp the 
        instantaneous desired speed slowly.
        """
        _dt_s = self._clock() - self._last_ramp_time
        self._last_ramp_time = self._clock()
    
        if self._desired_long_speed != self._gait_config.desired_long_speed:
            self._desired_long_speed =\
                min(self._desired_long_speed + _dt_s * cfg.ramp_long_acceleration_mpss, self._gait_config.desired_long_speed)

        if self._desired_lat_speed != self._gait_config.desired_lat_speed:
            self._desired_lat_speed = \
                min(self._desired_lat_speed + _dt_s * cfg.ramp_lat_acceleration_mpss, self._gait_config.desired_lat_speed)

        if self._desired_twisting_speed != self._gait_config.desired_ang_speed:
            self._desired_twisting_speed =\
                min(self._desired_twisting_speed + _dt_s * cfg.ramp_twisting_acceleration_radpss, self._gait_config.desired_ang_speed)

        self._desired_speed = [self._desired_long_speed, self._desired_lat_speed,0.0]

    @property
    def is_safe(self):
        if self.mode != ControllerMode.WALK:
            return True
        
        rot_mat = np.array(self._robot.pybullet_client.getMatrixFromQuaternion
                (self._state_estimator.com_orientation_quat_ground_frame)).reshape((3, 3))
        
        up_vec = rot_mat[2, 2]
        
        base_height = self._robot.base_position[2]
   
        return up_vec > cfg.up_vec_safe_threshold and base_height > cfg.base_height_safe_threshold
    
    @property
    def mode(self):
        return self._mode
    

if __name__ == "__main__":
    # Instantiate locomotion
    controller = LocomotionController()
