"""A gait generator that computes leg states based on sinusoids."""
import numpy as np
from typing import Any,List

from src.convex_mpc_controller import gait_generator
LegState = gait_generator.LegState

# Configuration parameters for this file
from src.convex_mpc_controller import controller_configs
cfg = controller_configs.get_offset_gait_generator_conf()

class OffsetGaitGenerator(gait_generator.GaitGenerator):
    
    """Phase-variable based gait generator."""
    def __init__(self,robot:Any,gait_params:List[float]):
        
        """Initializes the gait generator.
        Args:
            robot: Robot Instance
        """
        self.robot = robot
        self.current_phase = np.zeros(4)
        self._early_touchdown_phase_threshold = cfg.early_touchdown_phase_threshold
        self._lose_contact_phase_threshold = cfg.lose_contact_phase_threshold
        self.gait_params = gait_params
        self.prev_frame_robot_time = 0
        self.swing_cutoff = 2 * np.pi * 0.5
        
    def reset(self):
        self.current_phase = np.zeros(4)
        self.steps_since_reset = 0
        self.prev_frame_robot_time = self.robot.time_since_reset
        self.last_action_time = self.robot.time_since_reset
        self.swing_cutoff = 2 * np.pi * 0.5    
    
    def update(self):
        # Calculate the amount of time passed
        current_robot_time = self.robot.time_since_reset
        frame_duration = self.robot.time_since_reset - self.prev_frame_robot_time
        self.prev_frame_robot_time = current_robot_time
        
        # Propagate phase for front-right leg
        # Eq1 in https://arxiv.org/pdf/2104.04644.pdf
        # Remaining logic is as per Sec 3.2 of the paper
        
        self.current_phase[0] += 2 * np.pi * frame_duration * self.gait_params[0]
        
        # Offset for remaining legs
        self.current_phase[1:4] = self.current_phase[0] + self.gait_params[1:4]
        
        self.swing_cutoff = 2 * np.pi * self.gait_params[4]
        
        self.stance_duration = 1 / self.gait_params[0] * (1 - self.gait_params[4]) * np.ones(4)
        
    def get_estimated_contact_states(self, num_steps, dt):
        current_phase = self.current_phase.copy()
        future_phases = np.repeat(np.arange(num_steps)[:, None], 4,axis=-1) * 2 * np.pi * self.gait_params[0] * dt
        all_phases = np.fmod(current_phase + future_phases + 2 * np.pi, 2 * np.pi)
        ans = np.where(all_phases < self.swing_cutoff, False, True)
        return ans
    
    def get_observation(self):
        return np.concatenate(
            (np.cos(self.normalized_phase * np.pi),
            np.sin(self.normalized_phase * np.pi),
            np.where(
                np.fmod(self.current_phase, 2 * np.pi) < self.swing_cutoff, 0, 1),
            np.where(
                np.fmod(self.current_phase, 2 * np.pi) >= self.swing_cutoff, 0,
                1)))
    
    @property
    def desired_leg_state(self):
        modulated_phase = np.fmod(self.current_phase + 2 * np.pi, 2 * np.pi)
        return np.array([LegState.SWING if phase < self.swing_cutoff else LegState.STANCE for phase in modulated_phase])
    
    @property
    def normalized_phase(self):
        phase = np.fmod(self.current_phase + 2 * np.pi, 2 * np.pi)
        return np.where(phase < self.swing_cutoff, phase / self.swing_cutoff,(phase - self.swing_cutoff) /
            (2 * np.pi - self.swing_cutoff))

    @property
    def leg_state(self):
        leg_state = self.desired_leg_state.copy()
        contact_state = self.robot.foot_contacts

        for leg_id in range(self.robot.num_legs):
            if  (leg_state[leg_id] == gait_generator.LegState.STANCE
                    and not contact_state[leg_id] and
                    self.normalized_phase[leg_id] > self._lose_contact_phase_threshold):
                # print("Lose Contact Detected")
                leg_state[leg_id] = gait_generator.LegState.LOSE_CONTACT
                    
            if  (leg_state[leg_id] == gait_generator.LegState.SWING 
                    and contact_state[leg_id] and 
                    self.normalized_phase[leg_id] > self._early_touchdown_phase_threshold):
                # print("Early Touch Down Detected")
                leg_state[leg_id] = gait_generator.LegState.EARLY_CONTACT

        return leg_state