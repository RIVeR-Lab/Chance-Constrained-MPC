"""Helper enums and aliases"""

import enum
from src.worlds import plane_world, slope_world, stair_world, uneven_world

class ControllerMode(enum.Enum):
    DOWN = 1
    STAND = 2
    WALK = 3
    TERMINATE = 4
    
class GaitType(enum.Enum):
    TROT = 2
    FLYTROT = 3

class ConstraintTighteningMethod(enum.Enum):
    NONE=1 #Nominal MPC
    HEURISTIC=2 #Heuristic Safe MPC
    CHANCECONSTRAINTS=3 #ChanceConstrained MPC

WORLD_NAME_TO_CLASS_MAP = dict(plane=plane_world.PlaneWorld,
                                slope=slope_world.SlopeWorld,
                                stair=stair_world.StairWorld,
                                uneven=uneven_world.UnevenWorld)