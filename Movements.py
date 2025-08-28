from math import pi
from Commons.Constants import (
    DIST_BL,
    DIST_BR,
    DIST_BW,
    DIST_FL,
    DIST_FR,
    DIST_FW
)
from Commons.Utils import calc_vector
from Commons.Types import Position


D_THETA = pi/2


def forward(pos: "Position") -> "Position":
    new = pos.clone()
    new.add(calc_vector(pos.theta, DIST_FW))
    return new


def backward(pos: "Position") -> "Position":
    new = pos.clone()
    new.add(calc_vector(pos.theta, -DIST_BW))
    return new


def forward_left(pos: "Position") -> "Position":
    vh = calc_vector(pos.theta-pi/2, DIST_FL[0])
    vv = calc_vector(pos.theta, DIST_FL[1])
    new = pos.clone()
    new.add(vv + vh)
    new.theta = pos.theta + pi/2
    return new


def forward_right(pos: "Position") -> "Position":
    vh = calc_vector(pos.theta-pi/2, DIST_FR[0])
    vv = calc_vector(pos.theta, DIST_FR[1])
    new = pos.clone()
    new.add(vv + vh)
    new.theta = pos.theta - pi/2
    return new


def backward_left(pos: "Position") -> "Position":
    vh = calc_vector(pos.theta-pi/2, DIST_BL[0])
    vv = calc_vector(pos.theta, DIST_BL[1])
    new = pos.clone()
    new.add(vv + vh)
    new.theta = pos.theta - pi/2
    return new


def backward_right(pos: "Position") -> "Position":
    vh = calc_vector(pos.theta-pi/2, DIST_BR[0])
    vv = calc_vector(pos.theta, DIST_BR[1])
    new = pos.clone()
    new.add(vv + vh)
    new.theta = pos.theta + pi/2
    return new