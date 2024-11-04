# Lint as: python3

"""A torque based stance controller framework."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Tuple
import numpy as np
import pybullet as p  # pytype: disable=import-error
import scipy

from src.robots.motors import MotorCommand
from src.convex_mpc_controller import gait_generator as gait_generator_lib
from src.convex_mpc_controller.types import ConstraintTighteningMethod, GaitType

# Aliases
_stance = gait_generator_lib.LegState.STANCE
_early_contact = gait_generator_lib.LegState.EARLY_CONTACT
_lose_contact = gait_generator_lib.LegState.LOSE_CONTACT

try:
    import mpc_osqp as convex_mpc  
    import chance_constraints_cpp as cc
except Exception as e:  #pylint: disable=W0702
    print("Exception encountered when importing modules: {}".format(e))
    import sys
    print("Run python3 setup.py install --user in this repo")
    sys.exit()

# Configuration parameters for this file
from src.convex_mpc_controller import controller_configs
cfg = controller_configs.get_torque_stance_leg_controller_conf()

# Local copy of other configuration files
locomotion_controller_config = controller_configs.get_locomotion_controller_conf()

if locomotion_controller_config.gait_type == GaitType.TROT:
    gait_config = controller_configs.get_trot_conf()
elif locomotion_controller_config.gait_type == GaitType.FLYTROT:
    gait_config = controller_configs.get_flytrot_conf()
else:
    raise ValueError("Incorrect gait type")

class TorqueStanceLegController:
    """
    A torque based stance leg controller framework.

    Takes in high level parameters like walking speed and turning speed, and
    generates necessary the torques for stance legs.
    """
    def __init__(self,
                robot: Any,
                gait_generator: Any,
                state_estimator: Any,
                desired_speed: Tuple[float, float],
                desired_twisting_speed: float ,
                desired_body_height: float,
                planning_horizon_steps:float,
                planning_timestep:float,
                mpc_weights:Tuple[float,float,float,float,float,float,float,float,float]):
        
        """
        Initializes the class.
        
        Tracks the desired position/velocity of the robot by computing proper joint torques using MPC module.
        
        Args:
            robot: A robot instance.
            gait_generator: Used to query the locomotion phase and leg states.
            state_estimator: Estimate the robot states (e.g. CoM velocity).
            desired_speed: desired CoM speed in x-y plane.
            desired_twisting_speed: desired CoM rotating speed in z direction.
            desired_body_height: The standing height of the robot.
            planning_horizon_steps: MPC horizon
            planning_timestep: MPC dt between time steps in seconds
            mpc_weights: Tracking weights for MPC.
        """
        
        self._robot = robot
        self._gait_generator = gait_generator
        self._state_estimator = state_estimator
        self.desired_speed = desired_speed
        self.desired_twisting_speed = desired_twisting_speed
        self._planning_horizon_steps = planning_horizon_steps
        self._planning_timestep = planning_timestep
        
        self._desired_body_height = desired_body_height
        self._body_mass = cfg.mpc_mass
        self._num_legs = cfg.num_legs
        self._force_dimension = cfg.force_dimension
        
        self._friction_coeffs = np.array([cfg.mpc_surface_friction] * self._num_legs)
        
        if np.any(np.isclose(self._friction_coeffs, 1.)):
            raise ValueError("self._cpp_mpc.compute_contact_forces seg faults when a friction coefficient is equal to 1.")
        
        body_inertia_list = list(cfg.mpc_inertia)
        self._body_inertia_list = body_inertia_list

        weights_list = list(mpc_weights)
        self._weights_list = weights_list
        
        self._num_constraints_per_foot = cfg.num_friction_cone_surfaces+1

        # Control weights for all three force dimensions and all four feet
        self._control_weight_vec = gait_config.mpc_control_weights * cfg.num_legs

        self._cpp_mpc = convex_mpc.ConvexMpc(self._body_mass,
                                            self._body_inertia_list,
                                            self._num_legs,
                                            self._planning_horizon_steps,
                                            self._planning_timestep,
                                            self._weights_list,
                                            self._control_weight_vec,
                                            convex_mpc.QPOASES,
                                            cfg.fz_min_scale,
                                            cfg.fz_max_scale)
        
        self._future_contact_estimate = np.ones((planning_horizon_steps, 4))

        # Store MPC solution this iteration for constraint tightening next iteration
        self._previous_mpc_solution = None

        # Instantiate class to compute friction cone constraint tightening
        self._instantiate_chance_constraints_cpp()
        
        # Print out constraint tightening type
        if cfg.constraint_tightening_method == ConstraintTighteningMethod.NONE:
            print("------------------------------------------------")
            print("Running nominal linear mpc controller")
            print("------------------------------------------------")
        elif cfg.constraint_tightening_method == ConstraintTighteningMethod.CHANCECONSTRAINTS:
            print("------------------------------------------------")
            print("Running chance constrained mpc controller")
            print("------------------------------------------------")
        elif cfg.constraint_tightening_method == ConstraintTighteningMethod.HEURISTIC:
            print("------------------------------------------------")
            print("Running heuristic mpc controller")
            print("------------------------------------------------")
        else:
            raise NotImplementedError("Incorrect MPC type set. Please check constraint tightening method in configurations")
            

    def _instantiate_chance_constraints_cpp(self):
        
        """
        Create a ChanceConstraints object
        """

        dare_Q = cfg.dare_Q
        dare_R = cfg.dare_R
        constraint_tightening_multiplier_map = self._create_constraint_tightening_multiplier()
        sigma_theta = cfg.Sigma_theta
        sigma_w = cfg.Sigma_w
        mpc_horizon = self._planning_horizon_steps
        mpc_timestep = self._planning_timestep
        nominal_friction_coefficient = cfg.mpc_surface_friction
        robot_inertias = np.array((cfg.mpc_inerta_x, cfg.mpc_inerta_y,cfg.mpc_inerta_z))
        robot_mass = cfg.mpc_mass
        friction_cone_base_matrix = cfg.friction_cone_base_matrix

        self._chance_constraints_cpp = cc.ChanceConstraints(dare_Q,
                                                            dare_R,
                                                            constraint_tightening_multiplier_map,
                                                            sigma_theta,
                                                            sigma_w,
                                                            mpc_horizon,
                                                            mpc_timestep,
                                                            nominal_friction_coefficient,
                                                            robot_inertias,
                                                            robot_mass,
                                                            friction_cone_base_matrix)

    def _create_constraint_tightening_multiplier(self):
		# The number of constraints is dependent on the no of feet in contact
		# Each foot contributes 4 constraints for friction cone and 1 for fz.
        constraint_tightening_multiplier_map = {0:0.0}
        alpha = cfg.constraint_satisfaction_threshold

        for leg_no in range(self._num_legs):
            num_legs_in_contact = leg_no + 1
            num_constraints = self._num_constraints_per_foot * num_legs_in_contact  
            delta = (1.0-alpha) / (num_constraints)
            constraint_multiplier = scipy.stats.norm.ppf(1.0 - delta)
            constraint_tightening_multiplier_map[num_legs_in_contact] = constraint_multiplier
        
        return constraint_tightening_multiplier_map

    def reset(self, current_time):
        del current_time
        
        # Re-construct CPP solver to remove stochasticity due to warm-start
        self._cpp_mpc = convex_mpc.ConvexMpc(self._body_mass,
                                            self._body_inertia_list,
                                            self._num_legs,
                                            self._planning_horizon_steps,
                                            self._planning_timestep,
                                            self._weights_list,
                                            self._control_weight_vec,
                                            convex_mpc.QPOASES,
                                            cfg.fz_min_scale,
                                            cfg.fz_max_scale)
        
        # Reconstruct the chance constraint cpp object
        self._instantiate_chance_constraints_cpp()
        
        self._previous_mpc_solution = None
        
    def update(self, current_time, future_contact_estimate=None):
        del current_time
        self._future_contact_estimate = future_contact_estimate
        
    def get_action(self, desired_speed, desired_twisting_speed):
        """Computes the torque for stance legs.
        Computations are happening in the ground frame
        """
        # Assign desired speeds
        self.desired_speed = desired_speed; self.desired_twisting_speed = desired_twisting_speed
        
        # Desired quantities
        desired_com_position = np.array((0., 0., self._desired_body_height), dtype=np.float64)
        
        desired_com_velocity = np.array((self.desired_speed[0], self.desired_speed[1], 0.), dtype=np.float64)
        
        # Walk parallel to the ground
        desired_com_roll_pitch_yaw = np.zeros(3)
        
        desired_com_angular_velocity = np.array((0., 0., self.desired_twisting_speed), dtype=np.float64)

        foot_contact_state = np.array( [(leg_state in (_stance,_early_contact,_lose_contact))
                                        for leg_state in self._gait_generator.leg_state],dtype=np.int32 )
        
        if not foot_contact_state.any():
            return {}, None
        
        if self._future_contact_estimate is not None:
            contact_estimates = self._future_contact_estimate.copy()
            contact_estimates[0] = foot_contact_state
            
        else:
            contact_estimates = np.array([foot_contact_state] * self._planning_horizon_steps)
        
        self._current_contact_plan = contact_estimates
        
        # Sensed quantities
        com_position = np.array(self._state_estimator.com_position_ground_frame)
        com_roll_pitch_yaw = np.array(p.getEulerFromQuaternion(self._state_estimator.com_orientation_quat_ground_frame))
        com_roll_pitch_yaw[2] = 0
        
        gravity_projection_vec = np.array(self._state_estimator.gravity_projection_vector)
        
        predicted_contact_forces = [0] * self._num_legs * cfg.force_dimension
        
        p.submitProfileTiming("predicted_contact_forces")

        # Update function for chance constraint cpp object
        self._update_chance_constraints_cpp()
        
        # Solve the quadratic programming based mpc for computing desired contact forces
        predicted_contact_forces = self._cpp_mpc.compute_contact_forces(
            com_position, #com_position
            np.asarray(self._state_estimator.com_velocity_ground_frame,dtype=np.float64), #com_velocity
            np.array(com_roll_pitch_yaw, dtype=np.float64),  #com_roll_pitch_yaw
            gravity_projection_vec, # Normal Vector of ground
            self._robot.base_angular_velocity_body_frame, #com_angular_velocity
            np.asarray(contact_estimates,dtype=np.float64).flatten(), # Foot contact states
            np.array(self._robot.foot_positions_in_base_frame.flatten(),dtype=np.float64), #foot_positions_base_frame
            self._friction_coeffs, #foot_friction_coeffs
            desired_com_position, #desired_com_position
            desired_com_velocity, #desired_com_velocity
            desired_com_roll_pitch_yaw, #desired_com_roll_pitch_yaw
            desired_com_angular_velocity, #desired_com_angular_velocity
            self._constraint_tightening_flattened) #friction cone constraint tightening
        
        # Store back MPC solution for chance constraints computation at next iteration
        # Note the negative sign, since the optimization output is negated before sending 
        # joint torques
        self._previous_mpc_solution = [-val for val in predicted_contact_forces]

        p.submitProfileTiming()
        
        contact_forces = {}
        
        for i in range(self._num_legs):
            contact_forces[i] = np.array(predicted_contact_forces[i * self._force_dimension:(i + 1) *self._force_dimension])
        
        action = {}
        
        for leg_id, force in contact_forces.items():
            # While "Lose Contact" is useful in simulation, in real environment it's susceptible to sensor noise. Disabling for now.
            if self._gait_generator.leg_state[leg_id] == gait_generator_lib.LegState.LOSE_CONTACT and \
                cfg.disable_lose_contact==False:
                print("Disabling force")
                force = (0, 0, 0)
                
            motor_torques = self._robot.map_contact_force_to_joint_torques(leg_id, force)
            
            for joint_id, torque in motor_torques.items():
                action[joint_id] = MotorCommand(desired_position=0,
                                        kp=0,
                                        desired_velocity=0,
                                        kd=0,
                                        desired_extra_torque=torque)
        
        return action, contact_forces

    def _update_chance_constraints_cpp(self):
        """Return the constraint tightening for nominal/heuristic/chance constrained MPC

		Args:
            current_contact_plan (np.array) : Current contact plan for each foot across the MPC horizon
            previous_mpc_solution (np.array) : Control action at the previous time step 

		Raises:
            NotImplementedError: When a wrong constraint tightening method is set in the parent config file
		"""

        if cfg.constraint_tightening_method == ConstraintTighteningMethod.NONE or self._previous_mpc_solution == None:
            # Nominal MPC solution or first iteration on robot startup
            self._constraint_tightening_flattened = \
                np.zeros(self._num_legs * self._num_constraints_per_foot * self._planning_horizon_steps)
        
        elif cfg.constraint_tightening_method == ConstraintTighteningMethod.HEURISTIC:
            # Return the pre-configured constraint tightening for each friction cone surfaces
            # The values are projected across the MPC horizon and for each leg
            # Zeros for the feet not planned to be in contact
        
            contact_plan_reshaped = self._current_contact_plan.reshape(self._num_legs,self._planning_horizon_steps)

            output = np.zeros((self._num_legs, self._planning_horizon_steps,self._num_constraints_per_foot))

            output[contact_plan_reshaped] = cfg.heuristic_constraint_tightening_array

            self._constraint_tightening_flattened = output.reshape(-1)
            
        elif cfg.constraint_tightening_method == ConstraintTighteningMethod.CHANCECONSTRAINTS:
            reduced_B = self._cpp_mpc.get_previous_bmat()[6:12,:]
            foot_positions_flattened = self._cpp_mpc.get_previous_foot_positions_world().flatten()
            
            self._constraint_tightening_flattened = \
                self._chance_constraints_cpp.chance_constraint_tightening(reduced_B,
                                                                        foot_positions_flattened,
                                                                        self._previous_mpc_solution,
                                                                        self._current_contact_plan.flatten())
        else:
            raise NotImplementedError ("Incorrect constraint tightening method chosen")
        