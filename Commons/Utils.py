from Commons.Types import Position
from math import cos, sin, radians
import numpy as np


def euclidean(
    start: Position,
    end: Position
) ->  float:
    return ((start.y - end.y)**2 + (start.x - end.x)**2)**.5

# calculating a vector of a certain length (direction in radians)
# Theta - direction 
# Length - Magnitude 
# np.array - column vector (to calculate the vector going from robot's bottom left to top left corner)
def calc_vector(
    theta: float,
    length: float
) -> np.array:
    return np.array([
        cos(theta) * length,
        sin(theta) * length
    ])
