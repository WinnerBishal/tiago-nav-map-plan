import numpy as np
import math
import py_trees
import random
from scipy import signal
import matplotlib.pyplot as plt

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
    scale_x = 200/4.5
    scale_y = 300/6.5
    
    w_P_f = [-3.67/2 - 0.4, -0.17 + 2.25 + 0.2]
    
    xf = px/scale_x
    yf = py/scale_y
    
    xw = xf + w_P_f[0]
    yw = -yf + w_P_f[1]
    
    return (xw, yw)

class Mapping(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, blackboard):
        
        super(Mapping, self).__init__(name)
        
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
    
    def setup(self):
        
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)
        
        self.lidar = self.robot.getDevice("Hokuyo URG-04LX-UG01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        self.display = self.robot.getDevice("display")
        self.logger.debug("  %s [Mapping::setup()]" % self.name)
        
    
    def initialise(self):
        
        # Initialize map to store environment
        self.map = np.zeros((200, 300))
        
        # Initialize kernel to be used in C-Space calculation
        self.kernel = np.ones((27, 27))
        
        # Initialize range of angles of LiDAR measurement
        # 667 is the output no. of points by LiDAR
        
        self.angles = np.linspace(4.19/2, -4.19/2, 667)[80:-80]
        
        self.logger.debug("  %s [Mapping::initialise()]" % self.name)
        
    def update(self):
        
        # Getting position of robot at a timestep using gps data
        
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        
        # Orientation of robot using data from compass
        
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        if theta > np.pi:
            theta = theta - 2*np.pi
        
        # Obstacle points from LiDAR
        
        ranges = np.array(self.lidar.getRangeImage())
        ranges = [1000 if math.isinf(x) else x for x in ranges][80:-80]
        
        # pos(x, y) of all obstacle points in robot frame is calculated in one shot
        # correction in x position since LiDAR device is not at center
        # reshaping index is 667 - 160 since LiDAR cannot see on -80 to 80 degree range
        
        pos_r = np.array([[np.multiply(ranges, np.cos(self.angles)) + 0.202],
                     [np.multiply(ranges, np.sin(self.angles))]]).reshape(2, 667-160)
        
        # Since pos in pos_r is of form [x, y, 1]^T 
        pos_r = np.vstack([pos_r, np.ones((667-160, ))])
        
        # Formation of Transformation Matrix Representation for easier calculation 
        w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                      [np.sin(theta), np.cos(theta), yw],
                      [0, 0, 1]])
        
        pos_w = w_T_r @ pos_r
        
        # Probabilistic Mapping
        
        for i in range(len(pos_w[0])):
            pos_disp = world2map(pos_w[0][i], pos_w[1][i])
            
            px = pos_disp[0]
            py = pos_disp[1]
            
            if self.map[px][py] < 1:
                self.map[px][py] += 0.01
            
            v = int(self.map[px][py]*255)
            
            color_val = v*256**2 + v*256 + v
            self.display.setColor(color_val)
            self.display.drawPixel(pos_disp[0], pos_disp[1])
        
        if self.blackboard.read("nav_complete"):
            cmap = signal.convolve2d(self.map, self.kernel, mode = 'same')
            cspace = cmap > 0.9
            np.save('cspace', cspace)
            
            for x in range(cspace.shape[0]):
                for y in range(cspace.shape[1]):
                    
                    if cspace[x][y] > 0.9:
                        self.display.setColor(0xFFFFFF)
                        self.display.drawPixel(x, y)
                    else:
                        self.display.setColor(0x000000)
                        self.display.drawPixel(x, y)
                        
            print("Map Success")
            return py_trees.common.Status.SUCCESS
        # plt.imshow(cspace)
        # plt.show()
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.logger.debug("  %s [Mapping::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

        
        
                      
        