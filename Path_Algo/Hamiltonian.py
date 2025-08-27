import heapq
import logging
import multiprocessing as mp #run independent work processes in prarallel
import time
from typing import List

from Commons.Utils import euclidean
from Commons.Types import Position

logger = logging.getLogger('HAMILTONIAN PATH')

def k_nearest_neighour(mp : "Map", source : "Position" ) : 
    atsar = AStar(mp)
    path = [source] + [obstacle.to_pos() for obstacle in mp.obstacles]

    for i in range(1,len(path)) :
        nearest_dist = float('inf')
        nearest_dist_index = -1

        for j in range(i, len(path)) :
            dist = euclidean(path[i-1], path[j]) # distance from last fixed point to candidate j
            if dist < nearest_dist :
                nearest_dist = dist
                nearest_dist_index = j

        # swap the nearest neighbor with the current position
        if nearest_dist_index != -1:
            path[i], path[nearest_dist_index] = path[nearest_dist_index], path[i]

    result = []
    previous = source
    for i in range(1,len(path)) :
        result.append(astar.search(prev,path[i]))
        previous = result[-1][-1].c_pos
    return result # shortest path using greedy search 


