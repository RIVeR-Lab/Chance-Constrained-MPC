"""Build a world with uneven terrains."""
import numpy as np

# Configuration parameters for this file
from src.worlds import world_configs
cfg = world_configs.get_worlds_conf()

class UnevenWorld:
    def __init__(self, pybullet_client):
        self._pybullet_client = pybullet_client
        
    def build_world(self):
        p = self._pybullet_client
        height_perturbation_range = cfg.height_perturbation_range
        num_heightfield_rows = cfg.num_heightfield_rows
        num_heightfield_columns = cfg.num_heightfield_cols
        heightfield_data = [0] * num_heightfield_rows * num_heightfield_columns
        
        for j in range(int(num_heightfield_columns / 2)):
            for i in range(int(num_heightfield_rows / 2)):
                height = np.random.uniform(0, height_perturbation_range)
                heightfield_data[2 * i + 2 * j * num_heightfield_rows] = height
                heightfield_data[2 * i + 1 + 2 * j * num_heightfield_rows] = height
                heightfield_data[2 * i + (2 * j + 1) * num_heightfield_rows] = height
                heightfield_data[2 * i + 1 +(2 * j + 1) * num_heightfield_rows] = height
                
        terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[.1, .1, 1],  # <-- change this for different granularity
            heightfieldTextureScaling=(num_heightfield_rows - 1) / 2,
            heightfieldData=heightfield_data,
            numHeightfieldRows=num_heightfield_rows,
            numHeightfieldColumns=num_heightfield_columns)
        
        ground_id = p.createMultiBody(0, terrain_shape)
        p.changeDynamics(ground_id, -1, lateralFriction=cfg.uneven_lateral_friction)
        p.changeVisualShape(ground_id, -1, rgbaColor=[1, 1, 1, 1])
        return ground_id