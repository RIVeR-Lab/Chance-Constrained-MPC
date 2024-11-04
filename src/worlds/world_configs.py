"""Configurations for all the worlds"""

import ml_collections
from typing import List

def get_worlds_conf():
    """Configurations specific to all the simulation worlds"""
    config = ml_collections.ConfigDict()

    #####Plane World Configs#####
    config.plane_lateral_friction:float = 0.99

    #####Slope World Configs#####
    config.slope_lateral_friction:float = 10.
    config.slope_center_x:float = -1.25
    config.slope_center_z:float = -0.5
    config.slope_angle:float = -0.25
    config.slope_length:float=10
    config.slope_width:float=2

    #####Stair World Configs#####
    config.stair_lateral_friction:float = 10.
    config.num_steps:int = 30
    config.stair_height:float = 0.045
    config.stair_length:float = 0.25
    config.first_step_at:float = 0.7

    #####Slope World Configs#####
    config.uneven_lateral_friction:float = 1.
    config.num_heightfield_rows:int = 512
    config.num_heightfield_cols:int = 512
    config.height_perturbation_range:float = 0.06

    return config

def get_uncertain_world_conf():
    """Configurations specific to creating a robot world with additional unknown payload 
    and randomly placed wooden blocks"""
    config = ml_collections.ConfigDict()

    # Additional payload for single simulation
    config.payload_kg:float = 6.0

    # Dimensions of mass block, [length,breadth,height]
    config.payload_dimensions_lbh_m:List = [0.15,0.15,0.15]

    # Height from com to top of the robot
    config.height_from_com_to_top:float = 0.06
    
    ######### Planks Configurations#########
    
    # Number of planks
    config.number_of_planks:int = 10

    # Starting position of the first wooden block from robot
    config.first_plank_starting_pos_m:float = 0.7

    # Distance in meters between two successive wooden planks
    config.distance_between_planks_m:float = 0.50

    # Length of the planks
    config.plank_length_m:float = 0.2

    # Width of the planks
    config.plank_width_m:float = 2.

    # Height of the planks
    config.plank_height_m:float = 0.04

    # Lateral friction coefficient. Kept same for wooden blocks and ground
    config.lateral_friction:float = 1.

    return config