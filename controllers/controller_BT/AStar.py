import numpy as np
from heapq import heapify, heappop, heappush
from collections import defaultdict

class AStarPlanner():
  def __init__(self, map, start, goal):
    self.map = map
    self.start = start
    self.goal = goal
  
  def getNeighbors(self, u):
    neighbors = []
    cd = np.sqrt(2)

    for (c, delta) in ((1, (0, 1)), (1, (0, -1)), (1, (1, 0)), (1, (-1, 0)), (cd, (1, 1)), (cd, (1, -1)), (cd, (-1, 1)), (cd, (-1, -1))):
      cost, cand = (c, (u[0] + delta[0], u[1] + delta[1]))

      if cand[0] >= 0 and cand[0] < self.map.shape[0]:
        # print("First Loop")
        if cand[1] >= 0 and cand[1] < self.map.shape[1]:
          # print(f"Second Loop : {self.map[cand[0]][cand[1]]}")
          if self.map[cand[0]][cand[1]] < 1:
            # print("Third Loop")
            neighbors.append((cost, cand))
            
    return neighbors
  
  def plan(self):
    queue = [(0, self.start)]
    heapify(queue)
    
    distances = defaultdict(lambda: float('inf'))
    
    distances[self.start] = 0
    visited = {self.start}
    parent = {}

    
    while self.goal not in visited:
        # print(f"Queue: {queue}")
        if len(queue) == 0 :
            break
        (currentdist, v) = heappop(queue)
        # print(f'Neighbors of {v} : {self.getNeighbors(v)}, Map : {self.map.shape}')
        visited.add(v)
        for (costvu, u) in self.getNeighbors(v):
            if u not in visited:
                newcost = distances[v] + costvu
                heuristic_cost = np.sqrt((self.goal[0] - u[0])**2 + (self.goal[1] - u[1])**2)
                if newcost < distances[u]:
                    distances[u] = newcost
                    heappush(queue, (newcost + heuristic_cost, u))
                    parent[u] = v
    
    path = []
    key = self.goal
    while key in parent.keys():
        key = parent[key]
        path.insert(0, key)

    return path