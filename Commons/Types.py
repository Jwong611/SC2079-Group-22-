from math import pi
import numpy as np
from typing import Tuple

from Commons.Constants import SNAP_COORD, SNAP_THETA


class Position :

    def __init__(self, x: float, y :float, theta : float): 
        self.x = x
        self.y = y
        self.theta = theta


    def snap ( self,) -> "Position" :  # rounding to nearest grid
        snap_x = int(round(self.x / SNAP_COORD) * SNAP_COORD)
        snap_y = int(round(self.y / SNAP_COORD) * SNAP_COORD)
        theta = round(self.theta % (2*pi) / pi * 180 / SNAP_THETA) * SNAP_THETA / 180 * pi
        # wraps angle into [0, 2 pi) and converts it to degrees, then
        # converts degrees to a count of grid steps (quantize the angle)
        # then rounds to nearest integer number of steps and converts back to degrees
        # converts back to radians again

        return Position(snap_x, snap_y, theta)
    
    def getPosition(self) -> "Position" :
        return Position(self.x,self.y,self.theta)
    
    def add_to_pos(self, vec : np.array) :
        self.x += vec[0]
        self.y += vec[1]
        

    def getPositionArray(self) -> np.array :
        return np.array([self.x, self.y, self.theta])
    
    def getPositionTuple(self) -> Tuple[float,float,float] :
        return (self.x, self.y, self.theta)
    
    def getPositionString(self) -> str :
        return f"({self.x: .2f}, {self.y: .2f}, {self.theta: .2f})"

        
