from controller import Robot, Supervisor
import numpy as np
import math
from scipy import signal
import matplotlib.pyplot as plt

from os.path import exists

from py_trees.composites import Sequence, Parallel, Selector
from py_trees import common
from navigation_leaf import Navigation
from mapping_leaf import Mapping
from planning_leaf import Planning

import py_trees

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

WP = [(0.77, 0.17), (0.87, -1.09), (-0.13, -3.22),
      (-1.64, -2.54), (-1.56, -0.44), (-0.82, 0.35)]
WP = np.concatenate((WP, np.flip(WP, 0)), axis = 0)

# Blackboard implementation for data exchange between leaf nodes

class Blackboard:
    def __init__(self):
        self.data = {}
    
    def write(self, key, value):
        self.data[key] = value
    
    def read(self, key):
        return self.data.get(key)

"""
DoesMapExist Behaviour is implemented to check if the map file exists in the directory.
"""

class DoesMapExist(py_trees.behaviour.Behaviour):

    def update(self):
        map_status = exists("cspace.npy")
        if map_status:
            print("Mapping not necessary.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Mapping is required.")
            return py_trees.common.Status.FAILURE
    

blackboard = Blackboard()

blackboard.write('robot',robot)
blackboard.write('waypoints', WP)

"""
Below is the implementation of the behaviour tree using the child nodes planning, mapping, navigation and doesMapExist.
The py_trees composites like Sequence, Parallel and Selector are used to model the behaviour.

First, it will be checked whether a map already exist, if not, a Parallel node will be triggered to simultaneously navigate and map
the environment. If it is successful, the program will plan a path to lower corner of the kitchen and a navigate node will be
triggered to take the robot from current position to goal position. Similar sequence is performed to plan a path and
move the robot from lower let corner to near the sink in kitchen.

"""
tree = Sequence("Main", 
                children = [Selector("Perform Mapping or Use Old",
                                      children = [DoesMapExist('Check Old Map'),
                                                  Parallel("Perform Mapping",
                                                           policy = common.ParallelPolicy.SuccessOnAll(),
                                                           children = [Navigation('Go Around Table', blackboard),
                                                                       Mapping('Map Using LiDar', blackboard)]
                                                           )
                                                  ],
                                       memory = True
                                     ),
                            Planning("Plan to First Goal", blackboard, goal = [-1.52, -3.36]),
                            Navigation("Navigate to First Goal", blackboard),
                            Planning("Plan to First Goal", blackboard, goal = [1.01, 0.55]),
                            Navigation("Navigate to First Goal", blackboard)
                           ],
                memory = True
               )

tree.setup_with_descendants()

while(robot.step(timestep) != -1):
    tree.tick_once()