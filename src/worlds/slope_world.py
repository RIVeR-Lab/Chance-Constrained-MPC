"""Slope world"""

# Configuration parameters for this file
from src.worlds import world_configs
cfg = world_configs.get_worlds_conf()

class SlopeWorld:
    def __init__(self,
                pybullet_client):
        
        self._pybullet_client = pybullet_client
        self._slope_center_x = cfg.slope_center_x
        self._slope_center_z = cfg.slope_center_z
        self._slope_angle = cfg.slope_angle
        self._slope_length = cfg.slope_length
        self._slope_width = cfg.slope_width
        
    def build_world(self):
        p = self._pybullet_client
        ground_id = self._pybullet_client.loadURDF('plane.urdf')
        
        slope_collision_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self._slope_length / 2, self._slope_width / 2, 0.05])
        
        slope_visual_id = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self._slope_length / 2, self._slope_width / 2, 0.05])
        
        slope_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=slope_collision_id,
            baseVisualShapeIndex=slope_visual_id,
            basePosition=[self._slope_center_x, 0, self._slope_center_z],
            baseOrientation=p.getQuaternionFromEuler((0., self._slope_angle, 0.)))
        
        p.changeDynamics(slope_id, -1, lateralFriction=cfg.slope_lateral_friction)
        return ground_id