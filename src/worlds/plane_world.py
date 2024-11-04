"""Build a simple world with plane only."""

# Configuration parameters for this file
from src.worlds import world_configs
cfg = world_configs.get_worlds_conf()

class PlaneWorld:
    """Builds a simple world with a plane only."""
    def __init__(self, pybullet_client):
        self._pybullet_client = pybullet_client
        
    def build_world(self):
        """Builds world with a simple plane and custom friction."""
        ground_id = self._pybullet_client.loadURDF('plane.urdf')
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=cfg.plane_lateral_friction)
        return ground_id