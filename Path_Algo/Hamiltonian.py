import heapq
import logging
import multiprocessing as mp #run independent work processes in prarallel
import time
import queue
from typing import List
from Path_Algo import Astar

from Commons.Utils import euclidean
from Commons.Types import Position

logger = logging.getLogger('HAMILTONIAN PATH')

def k_nearest_neighour(mp : "Map", source : "Position" ) : 
    aStar = Astar(mp)
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
        result.append(Astar.search(previous,path[i]))
        previous = result[-1][-1].c_pos
    return result # shortest path using greedy search 

# Generate all permutations of [0, 1, ..., n-1]
def permutate(n: int, start_from_zero: bool) -> List[List[int]]:
    if n ==0 :
        return [[]] if not start_from_zero else []
    
    result = List[List[int]] = []
    used = [False] * n
    curr: List[int] = []

    def helper():
        if len(curr) == n:
            result.append(curr)
            return
        for i in range(n):
            if not used[i] :
                used[i] = True
                curr.append(i)
                helper()
                curr.pop()
                used[i] = False

    helper([])

    if start_from_zero:
        result = [p for p in result if p[0] == 0]
    return result


class SearchProcess(mp.Process):
    def __init__(
        self,
        pos: List["Position"],
        astar: Astar,
        todo: mp.Queue,
        done: mp.Queue,
        i:int
    ):
        super().__init__()
        self.astar = astar
        self.pos = pos
        self.todo = todo
        self.done = done
        self.i = i
        logger.info(f'Spawning P{i}')


    # search for the shortest path between two points
    def search(
        self,
        start: int,
        end: int
    ) -> float:
        logger.info(f'P{self.i} start search {start, end}')
        path = self.astar.search(self.pos[start], self.pos[end])
        return path[-1].f if path else 99999
        
    # run the process to get tasks from the todo queue and put results in the done queue
    def run(self):
        while 1:
            try:
                st, end = self.todo.get()
                self.done.put((st, end, self.search(st, end)))
            except:
                logger.info(f'P{self.i} finished')
                return


class ExhaustiveSearch:

    def __init__(
        self,
        map: "Map", 
        src: "Position",
        n: int = 8
    ):
        self.astar = Astar(map)
        self.src = src
        self.pos = [src] + [o.to_pos() for o in map.obstacles]
        self.n = n

    # obtains cost of pair of nodes from todo
    def search(self, top_n: int = 3):
        st = time.time()
        n = len(self.pos)
        m = int(n*n - n)
        perms = permutate(n, True)
        edges = [[0 for _ in range(n)] for _ in range(n)]
        todo = mp.Queue()
        done = mp.Queue()

        for r in range(n):
            for c in range(n):
                if r != c:
                    todo.put((r, c)) 

        for i in range(self.n):
            p = SearchProcess(self.pos, self.astar, todo, done, i)
            p.daemon = True
            p.start()

        while m:
            r, c, f = done.get()
            edges[r][c] = f
            logger.info(f'{r} -> {c} ({f})')
            m -= 1
        logger.info(f'Adj list completed in {time.time()-st} s')

        # get shortest path (lowest cost)
        h = []
        for i, perm in enumerate(perms):
            cost = sum([edges[perm[i]][perm[i+1]] for i in range(n-1)])
            heapq.heappush(h, (cost, perm)) # cheapest permutation godes first

        loc_mn_path = []
        loc_mn_f = float('inf')
        min_perm = []
        for _ in range(min(top_n, len(h))):

            path = []
            prev = self.pos[0]
            cost, perm = heapq.heappop(h)
            f = 0
            logger.info(f'Calculating path for {perm}')

            for i in range(1, n):
                segment = self.astar.search(prev, self.pos[perm[i]])

                if segment:
                    path.append(segment)
                    prev = segment[-1].c_pos
                    f += segment[-1].f
                else:
                    f += 99999

                if f > loc_mn_f:
                    break

            if f < loc_mn_f:
                loc_mn_f = f
                loc_mn_path = path
                min_perm = perm
            if f < 99999:
                return perm, path
        
        return min_perm, loc_mn_path


