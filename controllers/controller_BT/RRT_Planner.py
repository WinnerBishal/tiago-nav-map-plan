import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from copy import deepcopy
from heapq import heapify, heappop, heappush
from skimage.draw import line_nd, random_shapes

def IsPathOpen(map, a, b):
    line_ab = line_nd(a, b, integer = True)
    x = line_ab[0]
    y = line_ab[1]
    if map[a[1]][a[0]] > 0:
        return False
    
    if map[b[1]][b[0]] > 0:
        return False
    
    for idx in range(len(x)):
        if map[y[idx]][x[idx]] > 0:
            plt.plot(x[idx], y[idx], 'k.')
            return False
        else:
            plt.plot(x[idx], y[idx], 'k.', markersize = 3)
    return True

def AStar(graph, start, goal):
    queue = [(0, start)]
    heapify(queue)
    
    distances = defaultdict(lambda: float('inf'))
    
    distances[start] = 0
    visited = {start}
    parent = {}

    
    while queue:
        (currentdist, v) = heappop(queue)
        visited.add(v)

        if v == goal:
            break
        for (costvu, u) in graph[v]:
            if u not in visited:
                newcost = distances[v] + costvu
                heuristic_cost = np.sqrt((goal[0] - u[0])**2 + (goal[1] - u[1])**2)
                if newcost < distances[u]:
                    distances[u] = newcost
                    heappush(queue, (newcost + heuristic_cost, u))
                    parent[u] = v
    
    path = []
    key = goal
    while key in parent.keys():
        key = parent[key]
        path.insert(0, key)
    path.append(goal)
    
    return path 

class RRTPlanner:
    def __init__(self, map, qstart, qgoal, max_nodes = 1000):
        self.map = map
        self.qstart = qstart
        self.qgoal = qgoal
        self.max_nodes = max_nodes
        
    def plan(self):
        G = {self.qstart : []}
        qnew = self.qstart
        dq_new = 3
        parent = {}
        
        plt.imshow(self.map, cmap = 'Blues')
        plt.title("RRT followed by A*")
        # plt.ion()
        plt.plot(self.qstart[0], self.qstart[1], 'y*')
        plt.plot(self.qgoal[0], self.qgoal[1], 'y*')
        
        # No. of nodes
        k = 1
        while qnew != self.qgoal:
            if k > self.max_nodes:
                break
            qprev = deepcopy(qnew)
            
            # Draw a random node
            while(True):
                qrand = (np.random.randint(0, self.map.shape[1]), np.random.randint(0, self.map.shape[0]))
                if self.map[self.qgoal[1]][self.qgoal[0]] < 1:
                    break
            
            # Bias the algorithm towards goal with probability 
            if np.random.rand() < 0.2 :
                qrand = self.qgoal
                
            # Find the nearest node to random node from the graph
            distances = defaultdict(lambda:float('inf'))
            for node in G.keys():
                distances[node] = np.sqrt((node[0] - qrand[0])**2 + (node[1] - qrand[1])**2)
            nearest = min(zip(distances.values(), distances.keys()))
            
            # Assign nearest node
            qnear = nearest[1]
            dq_near = nearest[0]
            
            # If nearest node distance to random node is less than exploration distance, make the random node itself a new node
            if dq_near < dq_new:
                qnew = qrand
                dq_near = dq_new
            
            # Add a new node in the direction of qrand at a distance d_qnew
            a = np.asarray(qnear)
            b = np.asarray(qrand)
            
            qnew = (a + (b - a)/np.linalg.norm(b - a)*dq_new).astype(int)
            qnew = (qnew[0], qnew[1])
            
            # If goal is nearer to new node than random node, make the goal itself the new node
            distG = np.sqrt((qnew[0] - self.qgoal[0])**2 + (qnew[1] - self.qgoal[1])**2)
            if distG < dq_new :
                qnew = self.qgoal
                dq_near = distG
            
            # If new node is within the bounds, add all neighbors to the graph
            # Otherwise, set qnew to be same as previous and restart
            if qnew[0] >= 0 and qnew[0] < self.map.shape[1] and qnew[1] >= 0 and qnew[1] < self.map.shape[0] :
                if IsPathOpen(self.map, qnear, qnew):
                    
                    G[qnew] = [(dq_new, qnear)]
                    G[qnear].append((dq_new, qnew))
                        
                    parent[qnew] = qnear   
                    k += 1
                else : 
                    qnew = qprev
            
            # plt.plot(qnew[0], qnew[1], 'g.', markersize = 2)
            try:
                plt.plot([qnew[0], parent[qnew][0]], [qnew[1], parent[qnew][1] ], 'go-', markersize = 2, linewidth = 1)
            except:
                pass
            # plt.pause(0.001)
            # plt.show()
            # print(G)
        path = AStar(G, self.qstart, self.qgoal)
        for p in path:    
            plt.plot(p[0],p[1],'r*', markersize = 5, linewidth = 2)


        # plt.ioff()
        # plt.gca().invert_yaxis()
        
        # plt.show()  
        return path
