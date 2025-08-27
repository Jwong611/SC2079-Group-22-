from enum import Enum


class Direction(Enum):
    # Obstacle Image Facing
    NORTH = 1
    SOUTH = 2
    EAST = 3
    WEST = 4


class Movement(Enum):
    # 6 Possible Movements
    FWD = 1
    BWD = 2
    FWD_LEFT = 3
    FWD_RIGHT = 4
    BWD_LEFT = 5
    BWD_RIGHT = 6


class Turning(Enum):
    CLOCKWISE = 0
    ANTICLOCKWISE = 1


class Dubins_Path(Enum):
    LSL = 1
    LSR = 2
    RSL = 3
    RSR = 4
    LRL = 5
    RLR = 6
