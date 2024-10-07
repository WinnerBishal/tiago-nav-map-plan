import numpy as np
import math
import py_trees
from RRT_Planner import RRTPlanner

def world2map(xw, yw):
    # w_P_f = [-0.5 - 1.75, -3 - 1]          # floor origin at corner
    w_P_f = [-3.67/2 - 0.4, -0.17 + 2.25 + 0.2]
    # Coordinate transform
    xf = (xw - w_P_f[0])/4.5
    yf = -(yw - w_P_f[1])/6.5    # Since the frame is rotated
    
    # Scaling
    scale_x = 200
    scale_y = 300
    px = int(xf*scale_x)
    py = int(yf*scale_y)
    
    return [px if (px<scale_x and px>0) else scale_x - 1, py if (py<scale_y and py>0) else scale_y - 1]


def map2world(px, py):
    """
    Convert map coordinates (px, py) to world coordinates (px, py)
    """
    scale_x = 200/4.5
    scale_y = 300/6.5
    
    w_P_f = [-3.67/2 - 0.4, -0.17 + 2.25 + 0.2]
    
    xf = px/scale_x
    yf = py/scale_y
    
    xw = xf + w_P_f[0]
    yw = -yf + w_P_f[1]
    
    return (xw, yw)
    
"""
*** PLANNING BEHAVIOUR ***

The purpose of this behaviour is to load the map in .npy format from the directory and goal position from the blackboard,
use the current robot location to set the starting position,
then use RRT to sample the map to form a graph
then apply A* algorithm to find the optimal path from start to goal.

It writes the output path to the blackboard.
"""

class Planning(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, blackboard, goal):
        
        super(Planning, self).__init__(name)
        
        self.robot = blackboard.read("robot")
        self.blackboard = blackboard
        self.goal = world2map(goal[0], goal[1])
        
    
    def setup(self):
        self.logger.debug("  %s [Planning::setup()]" % self.name)    
        
    
    def initialise(self):
        self.map = np.load('cspace.npy')
        self.map = np.transpose(self.map)
        self.display = self.robot.getDevice("display")
        
        for x in range(self.map.shape[0]):
            for y in range(self.map.shape[1]):
                
                if self.map[x][y] > 0.9:
                    self.display.setColor(0xFFFFFF)
                    self.display.drawPixel(y, x)
                else:
                    self.display.setColor(0x000000)
                    self.display.drawPixel(y, x)
        self.display.setColor(0xBFFF00)
        self.display.drawText('Goal', self.goal[0], self.goal[1])
        self.blackboard.write("waypoints", [])
        self.path_in_world = []
        self.logger.debug("  %s [Planning::initialise()]" % self.name)
                
    def update(self):
        
        current_pos = self.robot.getFromDef("ROBOT").getField("translation")
        start = current_pos.getSFVec2f()
        start = world2map(start[0] + 0.3, start[1] - 0.3)
        
        planner = RRTPlanner(self.map, (start[0], start[1]), (self.goal[0], self.goal[1]))
        
        try:
            planned_path = planner.plan()
        except:
            return py_trees.common.Status.FAILURE
        
        for point in planned_path:
            self.path_in_world.append(map2world(point[0], point[1]))
        
        self.blackboard.write("waypoints", self.path_in_world)
        
        saved_path = self.blackboard.read("waypoints")
        
        # print(f"planning complete; Path Length : {len(planned_path)}; Start : {start}; Goal : {self.goal}")
        
        if len(saved_path) == 0:            
            return py_trees.common.Status.FAILURE
        
        elif len(saved_path) == len(planned_path):
            return py_trees.common.Status.SUCCESS
        
        else:
            return py_trees.common.Status.RUNNING
            
    def terminate(self, new_status):
        self.logger.debug("  %s [Planning::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
    
        
        
        
        
        
        