import math


INDOOR = True

# ROBOT DIMENSIONS

ROBOT_WIDTH = 25
ROBOT_HEIGHT = 28
ROBOT_ACTUAL_WIDTH = 18.8
ROBOT_VERT_OFFSET = (ROBOT_WIDTH - ROBOT_ACTUAL_WIDTH) / 2
ROBOT_MIN_CAMERA_DIST = 20


# OBSTACLE DIMENSIONS
OBSTACLE_WIDTH = 10
IMG_THICKNESS = 2
EDGE_ERR = 0.1
CONE = [10, 10, 4, 40]

# A STAR
_DIST_STR = 5


# MAP DIMENSIONS

TK_SCALE = 4  # 1 map unit = 4 screen pixels 
MAP_WIDTH = 200  # in map units
MAP_HEIGHT = 200
GRID_WIDTH = 5  # spacing of the display grid
SNAP_COORD = _DIST_STR # coordinate grid size to snap positions (rounded to multiples of 5)
SNAP_THETA = 15 # theta snapping to 0,15,30 etc...

# BUFFERS (buffer of 5 units in case of collision)
WIDTH_GRADIENT = ROBOT_WIDTH+5 
HEIGHT_GRADIENT = ROBOT_HEIGHT+5

