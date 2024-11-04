"""Stair world"""

# Configuration parameters for this file
from src.worlds import world_configs
cfg = world_configs.get_worlds_conf()

class StairWorld:
    def __init__(self,
                pybullet_client):
        
        self._pybullet_client = pybullet_client
        self._num_steps = cfg.num_steps
        self._stair_height = cfg.stair_height
        self._stair_length = cfg.stair_length
        self._first_step_at = cfg.first_step_at
        
    def build_world(self):
        """Builds world with stairs."""
        p = self._pybullet_client
        ground_id = self._pybullet_client.loadURDF('plane.urdf')
        
        stair_collision_id = p.createCollisionShape(p.GEOM_BOX,
                        halfExtents=[self._stair_length / 2, 1, self._stair_height / 2])
        
        stair_visual_id = p.createVisualShape(p.GEOM_BOX,
                        halfExtents=[self._stair_length / 2, 1, self._stair_height / 2])
        
        curr_x, curr_z = self._first_step_at, self._stair_height / 2
        
        for _ in range(self._num_steps):
            stair_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=stair_collision_id,
                                        baseVisualShapeIndex=stair_visual_id,
                                        basePosition=[curr_x, 0, curr_z])
            
            p.changeDynamics(stair_id, -1, lateralFriction=cfg.stair_lateral_friction)
            curr_x += self._stair_length
            curr_z += self._stair_height
        
        return ground_id
        