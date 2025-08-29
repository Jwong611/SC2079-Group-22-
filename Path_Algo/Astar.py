from typing import List, Optional
from Commons.Types import Position
from Grid.Map import Map
from Commons.Enums import Movement
from math import pi
from Commons.Utils import calc_vector, euclidean
import numpy as np
import heapq
import logging
from Robot_Movements.Movements import (
    forward,
    backward,
    forward_left,
    forward_right,
    backward_left,
    backward_right
)
from Commons.Constants import (
    DIST_BL, 
    DIST_BR, 
    DIST_BW, 
    DIST_FL, 
    DIST_FR, 
    DIST_FW,
    PENALTY_STOP,
    MAX_THETA_ERR,
    MAX_X_ERR,
    MAX_Y_ERR,
    WPS_FL,
    WPS_FR, 
    WPS_BL, 
    WPS_BR,
)

logger = logging.getLogger('ASTAR')

class Node :
        __slots__ = ("pos", "c_pos", "g", "h", "v", "s", "d", "parent")

        # pos : discrete position
        # c_pos :continous position
        # g : path cost from start to a node
        # h : heuristic cost from node to goal
        # f : g+h
        # v : previous motions direction (-1 : backwards, 1 : forwards)
        # s : previous steering direction (-1 : left, 0 : straight, 1 : right)
        # d :distance traveled in last step
        # parent : pointer to the previous node

        def __init__(self,
                    pos :"Position", c_pos : "Position", g : float,
                    h : float, parent : Optional["Node"], v : int = 1,
                    s : int = 0, d : float = 0.0):
                        self.pos = pos
                        self.c_pos = c_pos
                        self.g = g
                        self.h = h
                        self.v = v
                        self.s = s
                        self.d = d
                        self.parent = parent
        @property
        def f(self) -> float :
                return self.g + self.h
        
        def cloneNode(self) -> "Node" : # returns a copy of the node
                return Node(self.pos, self.c_pos, self.g, self.h, self.parent, self.v, self.s, self.d)
        
        def isEqual(self,node : "Node") -> bool :
                return (self.pos.x, self.pos.y, self.pos.theta) == (node.pos.x, node.pos.y, node.pos.theta)
        
        def lessThan(self,node : "Node") -> bool :
                return (self.f, self.h, self.pos.x, self.pos.y, self.pos.theta) < \
               (node.f, node.h, node.pos.x, node.pos.y, node.pos.theta)
        
        def hashNode(self) -> int : # hashes node
                return hash((self.pos.x, self.pos.y, self.pos.theta))
        
        def toString(self) -> str :
                return (f"Node(x:{self.c_pos.x:6.2f}, y:{self.c_pos.y:6.2f}, θ:{self.c_pos.theta:6.2f}, "
                f"g:{self.g:6.2f}, h:{self.h:6.2f}, f:{self.f:6.2f}), v:{self.v}, s:{self.s}")
                


class Astar :
        def __init__(self, mp : "Map") :
                self.moves = (
                        (-1,  0, DIST_BW,    Movement.BWD,       backward),
                        (-1, -1, DIST_BL[2], Movement.BWD_LEFT,  backward_left),
                        (-1,  1, DIST_BR[2], Movement.BWD_RIGHT, backward_right),
                        ( 1,  0, DIST_FW,    Movement.FWD,       forward),
                        ( 1, -1, DIST_FL[2], Movement.FWD_LEFT,  forward_left),
                        ( 1,  1, DIST_FR[2], Movement.FWD_RIGHT, forward_right),)
                self.map = mp
                self.end = None
                self.x_bounds = None
                self.y_bounds = None

        # Check if the position is within the goal bounds andif the heading difference is within the error theta
        def goal(self, pos : "Position") -> bool :
                return self.x_bounds[0] <= pos.x <= self.x_bounds[1] and \
                       self.y_bounds[0] <= pos.y <= self.y_bounds[1] and \
                       abs(self.end.theta - pos.theta) % (2 * pi) <= MAX_THETA_ERR
        
        def has_collision(self, start : Position, movement : Movement) -> bool :
                mp = self.map
                obs = mp.priority_obs(start, movement)

                if movement == Movement.FWD_LEFT:
                        wps = WPS_FL
                elif movement == Movement.BWD_LEFT:
                        wps = WPS_BL
                elif movement == Movement.FWD_RIGHT:
                        wps = WPS_FR
                elif movement == Movement.BWD_RIGHT:
                        wps = WPS_BR
                else :
                        v = calc_vector(start.theta, DIST_FW)
                        if movement == Movement.BWD :
                                v *= -1
                        new = start.cloneNode()
                        new.add(v)
                        return mp.is_valid(new,obs)
                
                start_vector = np.array([start.x, start.y])
                forward_vector = calc_vector(start.theta, 1)
                # backward is negative

                left_vector = calc_vector(start.theta - pi/2, 1)
                # right is negative

                # returns true if there is a collision and false otherwise
                for lx, ly, dth in wps :
                        pos = Position(*(start_vector + lx * left_vector + ly * forward_vector), (start.theta + dth) % (2*pi))
                        if not mp.is_valid(pos, obs):
                                return True
                return False


        
        def expand(self, node : "Node") :
                current_pos = node.c_pos

                for v, s, d, mv, f in self.moves :
                        next_pos_continous = f(current_pos)
                        next_pos_snap = next_pos_continous.snap() #snap continous into discrete
                        next_tuple = next_pos_snap.to_tuple() # hashable
                        
                        # checking successor if its expanded or will collide
                        if next_tuple in self.closed or self.has_collision(current_pos,mv,self.map) :
                                continue
                        # penalty for changing motion
                        penalty = PENALTY_STOP if (v != node.v or s != node.s) else 0

                        # update path cost
                        g =  node.g + penalty + d
                        h = euclidean(next_pos_continous, self.end)

                        # building sucessor node
                        next_node = Node(next_pos_snap, next_pos_continous, g, h, node, v, s, d)

                        # self.open : priority queue of nodes ordered by cost (f)
                        # look up the smallest f we've seen for this cell, if there is a better f, then update disctionary index self.open_h
                        best = self.open_h.get(next_tuple)
                        if best is None or next_node.f < best :
                                self.open_h[next_tuple] = next_node.f
                                heapq.heappush(self.open, next_node)


        def set_bounds(self):
                vv = calc_vector(self.end.theta, 1)
                vh = calc_vector(self.end.theta - pi/2, 1)

                end = np.array([self.end.x, self.end.y])
                _TR = end + vh * MAX_X_ERR[1] + vv * MAX_Y_ERR[0]
                _BL = end - vh * MAX_X_ERR[0] - vv * MAX_Y_ERR[1]

                self.x_bounds = sorted([_TR[0], _BL[0]])
                self.y_bounds = sorted([_TR[1], _BL[1]])


        def search(
                self,
                start: "Position",
                end: "Position",
                ) -> List["Node"]:

                logger.info(f'Start search from {start} to {end}')
                end_node = Node(end, end, 0, 0)
                self.end = end
                self.open = [Node(start.snap(), start,0, 0)]
                self.open_h = {} # index dictionary to remember best f per discrete cell
                self.closed = [] # list of cells already expanded
                self.set_bounds()

                while self.open:
                        node = heapq.heappop(self.open) # pop node with lowest f
                        tup = node.pos.to_tuple()
                        logger.debug(f'{node} {node.parent}')

                        if self.goal(node.c_pos):
                                logger.info(f'Found goal {end_node}')
                                return self._reconstruct(node)

                        self.closed.append(tup) # move to closed
                        self.expand(node) 

                        for o in self.open[:5]:
                                logger.debug(f'{o.c_pos}, {o}')

                logger.info(f'Unable to reach {end} from {start}')
                return []


        def reconstruct(  # constructing path from start to goal in result
                self,
                last: "Node"
                ) -> List["Node"]:
                result = []

                while last:
                        result.append(last)
                        last = last.parent

                return result[::-1]


        
        
