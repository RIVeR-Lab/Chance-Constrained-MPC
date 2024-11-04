"""Base class for all robots."""
import numpy as np
from typing import Any
from typing import Optional

from src.robots import kinematics
from src.robots.motors import MotorCommand
from src.robots.motors import MotorControlMode
from src.robots.motors import MotorGroup

# Configuration parameters for this file
from src.convex_mpc_controller import controller_configs
cfg = controller_configs.get_robot_conf()

# Configurations for uncertain world creation
from src.worlds import world_configs
uncertain_world_cfg = world_configs.get_uncertain_world_conf()

class Robot:
    """Robot Base
    
    A `Robot` requires access to a pre-instantiated `pybullet_client` and
    information about how the simulator was configured.
    
    A `Robot` is composed of joints which correspond to motors. For the most
    flexibility, we choose to pass motor objects to the robot when it is
    constructed. This allows for easy config-driven instantiation of
    different robot morphologies.
    
    Motors are passed as a collection of another collection of motors. A
    collection of motors at the lowest level is implemented in a `MotorGroup`.
    This means a robot can support the following configurations:
    
    1 Motor Robot: [ [ Arm Motor ] ]
    1 MotorGroup Robot: [ [ Arm Motor1, Arm Motor2 ] ]
    2 MotorGroup Robot: [ [ Leg Motor1, Leg Motor2 ],
      [ Arm Motor1, Arm Motor2 ] ]
    """
    
    def __init__(self,pybullet_client: Any,motors: MotorGroup):
                
        """Constructs a base robot and resets it to the initial states."""
        self._pybullet_client = pybullet_client
        self._motor_group = motors
        self._base_joint_names = ()
        self._foot_joint_names = cfg.foot_joint_names
        self._urdf_path = cfg.urdf_path
        self._num_motors = self._motor_group.num_motors if self._motor_group else 0
        self._motor_torques = None
        
        self._load_robot_urdf(self._urdf_path)
        self._step_counter = 0
        
        # Initialize the foot contact history to all feet on the ground at the first time step
        self._foot_contact_history = self.foot_positions_in_base_frame.copy()
        self._foot_contact_history[:, 2] = -self.mpc_body_height
        
        self._last_timestamp = 0
        self.reset()
    
    def _update_contact_history(self):
        """
        Update the contact location for each foot as either a new contact location
        or back integrated to the last time contact happened
        """
        dt = self.time_since_reset - self._last_timestamp
        self._last_timestamp = self.time_since_reset
        
        base_orientation = self.base_orientation_quat
        rot_mat = self.pybullet_client.getMatrixFromQuaternion(base_orientation)
        rot_mat = np.array(rot_mat).reshape((3, 3))
        
        base_vel_body_frame = rot_mat.T.dot(self.base_velocity)
        
        foot_contacts = self.foot_contacts.copy()
        foot_positions = self.foot_positions_in_base_frame.copy()
        
        for leg_id in range(4):
            if foot_contacts[leg_id]:
                self._foot_contact_history[leg_id] = foot_positions[leg_id]
            else: #if foot not in contact, go back by the body velocity 
                self._foot_contact_history[leg_id] -= base_vel_body_frame * dt
    
    def _apply_action(self,action,motor_control_mode=None):
        """Apply action in simulation"""
        # (post processed torques, raw desired torques)
        torques, observed_torques = self._motor_group.convert_to_torque(action, \
            self.motor_angles, self.motor_velocities, motor_control_mode)
        
        # Apply the torques in simulation
        self._pybullet_client.setJointMotorControlArray(bodyIndex=self.quadruped,jointIndices=self._motor_joint_ids,\
                                                        controlMode=self._pybullet_client.TORQUE_CONTROL,forces=torques,)
        
        self._motor_torques = observed_torques
    
    def step(self, action, motor_control_mode=None) -> None:
        """Step in simulation"""
        self._step_counter += 1
        for _ in range(cfg.action_repeat):
            self._apply_action(action, motor_control_mode)
            self._pybullet_client.stepSimulation()
            self._update_contact_history()
    
    def reset(self, hard_reset: bool = False, num_reset_steps: Optional[int] = None) -> None:
        
        """Resets the robot"""
        print("Resetting the robot in simulation")

        if hard_reset:
            # This assumes that resetSimulation() is already called.
            self._load_robot_urdf(self._urdf_path)
            
        else:
            init_position = (cfg.init_rack_position if cfg.on_rack else cfg.init_position)
            
            # Set the center of mass location/orientation initially in simulation
            self._pybullet_client.resetBasePositionAndOrientation(self.quadruped, init_position, [0.0, 0.0, 0.0, 1.0])
        
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        
        # Set all motors to have zero velocity in simulation
        for joint_id in range(num_joints):
            self._pybullet_client.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=(joint_id),\
                controlMode=self._pybullet_client.VELOCITY_CONTROL,targetVelocity=0,force=0,)
        
        # Set all motors to the desired initial joint angle in simulation
        for i in range(len(self._motor_joint_ids)):
            self._pybullet_client.resetJointState(self.quadruped,self._motor_joint_ids[i],\
                                                  self._motor_group.init_positions[i],targetVelocity=0,)
        
        # Steps the robot with position command
        if num_reset_steps is None:
            num_reset_steps = int(cfg.reset_time_s / cfg.timestep)
        
        # Standing pose control motor commands
        motor_command = MotorCommand(desired_position=self._motor_group.init_positions)
            
        for _ in range(num_reset_steps):
            # On the real robot, calls go1robot step function
            self.step(motor_command, MotorControlMode.POSITION)
        self._last_timestamp = self.time_since_reset
        self._step_counter = 0

    @property
    def foot_contacts(self):
        """Get foot contact states in simulation"""
        all_contacts = self._pybullet_client.getContactPoints(bodyA=self.quadruped)

        contacts = [False, False, False, False]
        for contact in all_contacts:
            # Ignore self contacts
            if contact[2] == self.quadruped:
                continue
            try:
                # From the links in contact check which toe is in stance phase
                toe_link_index = self._foot_link_ids.index(contact[3])
                contacts[toe_link_index] = True
            except ValueError:
                continue
        return contacts
    
    @property
    def motor_velocities(self):
        """Sim motor velocities"""
        joint_states = self._pybullet_client.getJointStates(self.quadruped, self._motor_joint_ids)
        return np.array([s[1] for s in joint_states])
      
    @property
    def motor_angles(self):
        """Sim motor angles"""
        joint_states = self._pybullet_client.getJointStates(self.quadruped, self._motor_joint_ids)
        return np.array([s[0] for s in joint_states])
        
    @property
    def base_position(self):
        """Sim base position"""
        return np.array(self._pybullet_client.getBasePositionAndOrientation(self.quadruped)[0])
    
    @property
    def base_velocity(self):
        """Sim base velocity"""
        return self._pybullet_client.getBaseVelocity(self.quadruped)[0]
    
    @property
    def base_orientation_rpy(self):
        """Sim orientation rpy"""
        return self._pybullet_client.getEulerFromQuaternion(self.base_orientation_quat)
    
    @property
    def base_orientation_quat(self):
        """Sim orientation quat"""
        return np.array(self._pybullet_client.getBasePositionAndOrientation(self.quadruped)[1])
    
    @property
    def motor_torques(self):
        """Measured motor torques in simulation"""
        return self._motor_torques

    @property
    def foot_contact_history(self):
        """Foot contact history"""
        return self._foot_contact_history
    
    def get_motor_angles_from_foot_position(self, leg_id, foot_local_position):
        """Gets the motor angle data for the foot with the given leg_id (and foot_position in base frame)"""
        # Foot id
        toe_id = self._foot_link_ids[leg_id]

        motors_per_leg = self.num_motors // self.num_legs
        
        # The 3 feet joint ids
        joint_position_idxs = list(range(leg_id * motors_per_leg,leg_id * motors_per_leg + motors_per_leg))

        # Helper function to get the relevant joint angles from inverse kinematics
        joint_angles = kinematics.joint_angles_from_link_position(robot=self,link_position=foot_local_position,\
                                                                  link_id=toe_id,joint_ids=joint_position_idxs,)
        
        # Return the joing index (the same as when calling GetMotorAngles) as well as the angles.
        return joint_position_idxs, joint_angles
    
    @property
    def base_angular_velocity_body_frame(self):
        """Simulation base angular velocity body frame"""
        # Angular velocity (B->W)
        angular_velocity = self._pybullet_client.getBaseVelocity(self.quadruped)[1]
        
        # Orientation (B->W)
        orientation = self.base_orientation_quat
        
        # Orientation (W->B)
        _, orientation_inversed = self._pybullet_client.invertTransform([0, 0, 0], orientation)
        
        
        relative_velocity, _ = self._pybullet_client.multiplyTransforms([0, 0, 0],orientation_inversed,
                                                        angular_velocity,self._pybullet_client.getQuaternionFromEuler([0, 0, 0]),)
        
        return np.asarray(relative_velocity)
            
    @property
    def foot_positions_in_base_frame(self):
        """Simulation foot positions in base frame"""
        foot_positions = []
        for foot_id in self._foot_link_ids:
            foot_positions.append(kinematics.link_position_in_base_frame(robot=self,link_id=foot_id,))

        return np.array(foot_positions)
    
    def _load_robot_urdf(self, urdf_path: str) -> None:
        if not self._pybullet_client:
            raise AttributeError("No pybullet client specified!")
        
        p = self._pybullet_client
        
        if cfg.on_rack:
            self.quadruped = p.loadURDF(urdf_path, cfg.init_rack_position, useFixedBase=True)
        else:
            self.quadruped = p.loadURDF(urdf_path, cfg.init_position)
    
        self._build_urdf_ids()
        
        # attach additional payload on the robot
        self._attach_additional_payload()
    
    def place_wooden_planks(self):

        p = self._pybullet_client

        plank_height_m = uncertain_world_cfg.plank_height_m

        number_of_planks = uncertain_world_cfg.number_of_planks

        first_plank_starting_pos_m = uncertain_world_cfg.first_plank_starting_pos_m

        distance_between_planks_m = uncertain_world_cfg.distance_between_planks_m

        lateral_friction = uncertain_world_cfg.lateral_friction

        robot_init_pos = cfg.init_position

        plank_start_x = robot_init_pos[0] + first_plank_starting_pos_m
        
        plank_positions = [(plank_start_x + i * distance_between_planks_m, robot_init_pos[1]) for i in range(number_of_planks)]
        
        wood_color = [0.76, 0.60, 0.42, 1]  # RGBA values for a wooden color

        for plank_number in range(number_of_planks):
            plank_id = p.createMultiBody(baseMass = 100,
						baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
												halfExtents=[uncertain_world_cfg.plank_length_m/2,uncertain_world_cfg.plank_width_m/2,plank_height_m/2]),
						baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
											   halfExtents=[uncertain_world_cfg.plank_length_m/2,uncertain_world_cfg.plank_width_m/2,plank_height_m/2],
											   rgbaColor=wood_color),
						basePosition=[plank_positions[plank_number][0],plank_positions[plank_number][1],plank_height_m/2])

            p.changeDynamics(plank_id,-1,lateralFriction=lateral_friction)
    
    def _attach_additional_payload(self):
        payload_kg = uncertain_world_cfg.payload_kg

        payload_dimensions = uncertain_world_cfg.payload_dimensions_lbh_m

        payload_half_extents = [size/2 for size in payload_dimensions]
		
        # Initial position on the ground
        robot_init_pos = cfg.init_position

        height_from_com_to_top = uncertain_world_cfg.height_from_com_to_top

        payload_init_pos = [robot_init_pos[0], robot_init_pos[1], robot_init_pos[2] + height_from_com_to_top + payload_half_extents[2]]

        payload_color_rgba = [139/255, 69/255, 19/255, 1]  # wooden payload

        # Create payload
        collision_shape_id = self._pybullet_client.createCollisionShape(self._pybullet_client.GEOM_BOX,halfExtents = payload_half_extents)

        visual_shape_id = self._pybullet_client.createVisualShape(self._pybullet_client.GEOM_BOX,\
																rgbaColor = payload_color_rgba,\
																halfExtents = payload_half_extents)

        payload_id = self._pybullet_client.createMultiBody(baseMass = payload_kg,\
														baseCollisionShapeIndex = collision_shape_id,\
														baseVisualShapeIndex = visual_shape_id,\
														basePosition = payload_init_pos)

        # Create a fixed constraint between robot and the payload
        self._pybullet_client.createConstraint(parentBodyUniqueId=self.quadruped,\
											parentLinkIndex=-1,  # -1 for the base
											childBodyUniqueId = payload_id,
											childLinkIndex=-1,  # -1 means no specific link of the block
											jointType=self._pybullet_client.JOINT_FIXED,
											jointAxis=[0, 0, 0],
											parentFramePosition=[0.,0.,height_from_com_to_top],
											childFramePosition=[-0.03,0.,-payload_half_extents[2]])

    def _build_urdf_ids(self) -> None:
        """Records ids of base link, foot links and motor joints.

        For detailed documentation of links and joints, please refer to the
        pybullet documentation.
        """
        self._chassis_link_ids = [-1]
        self._motor_joint_ids = []
        self._foot_link_ids = []
        
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        
        for joint_id in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, joint_id)
            joint_name = joint_info[1].decode("UTF-8")
            
            if joint_name in self._base_joint_names:
                self._chassis_link_ids.append(joint_id)
            elif joint_name in self._motor_group.motor_joint_names:
                self._motor_joint_ids.append(joint_id)
            elif joint_name in self._foot_joint_names:
                self._foot_link_ids.append(joint_id)
    
    def map_contact_force_to_joint_torques(self, leg_id, contact_force):
        """Maps the foot contact force to the leg joint torques."""
        
        jv = self.compute_foot_jacobian(leg_id)
        motor_torques_list = np.matmul(contact_force, jv)
        
        # Dict: joint id -> torque command
        motor_torques_dict = {}
        motors_per_leg = self.num_motors // self.num_legs
        
        for torque_id, joint_id in enumerate(range(leg_id * motors_per_leg, (leg_id + 1) * motors_per_leg)):
            motor_torques_dict[joint_id] = motor_torques_list[torque_id]
        
        return motor_torques_dict

    def compute_foot_jacobian(self, leg_id):
        """Compute the Jacobian for a given leg."""
        full_jacobian = kinematics.compute_jacobian(robot=self,link_id=self._foot_link_ids[leg_id],)
        
        motors_per_leg = self.num_motors // self.num_legs
        com_dof = 6

        return full_jacobian[:, com_dof + leg_id * motors_per_leg:com_dof +(leg_id + 1) * motors_per_leg]
    
    def send_rest_actions(self,last_sent_action:MotorCommand):
        pass

    @property
    def control_timestep(self):
        return cfg.timestep * cfg.action_repeat
    
    @property
    def time_since_reset(self):
        return self._step_counter * self.control_timestep
    
    @property
    def motor_group(self):
        return self._motor_group

    @property
    def num_legs(self):
        return 4

    @property
    def num_motors(self):
        raise NotImplementedError()

    @property
    def swing_reference_positions(self):
        raise NotImplementedError()

    @property
    def pybullet_client(self):
        return self._pybullet_client

    @property
    def mpc_body_height(self):
        raise NotImplementedError()

    @property
    def mpc_body_mass(self):
        raise NotImplementedError()

    @property
    def mpc_body_inertia(self):
        raise NotImplementedError()