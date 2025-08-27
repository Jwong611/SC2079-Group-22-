from math import pi

from Commons.Constants import (OBSTACLE_WIDTH, ROBOT_MIN_CAMERA_DIST, ROBOT_HEIGHT, ROBOT_WIDTH)
from Commons.Enums import Direction
from Commons.Types import Position


class Obstacle :
    def __init__(self, x : float, y : float, facing : Direction) : 
        self.x = x
        self.y = y 
        self.facing = facing
        self.middle = (x + OBSTACLE_WIDTH/2, y + OBSTACLE_WIDTH/2) #obstacle center point



    def to_pos(self) -> Position :
        x = self.x
        y = self.y
        theta = None 

        centering_pad = (ROBOT_WIDTH - OBSTACLE_WIDTH)/ 2

        if self.facing == Direction.NORTH :
            y += ROBOT_MIN_CAMERA_DIST + OBSTACLE_WIDTH + ROBOT_HEIGHT
            x += centering_pad + OBSTACLE_WIDTH
            theta = -pi/2
