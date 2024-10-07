import numpy as np
import math

import py_trees
import random

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

"""
***Navigation Behaviour****

The purpose of this behaviour is to read the Waypoints from blackboard
and make the robot follow those waypoints using a PID controller 
based on the odometry and gps data for accurate localization.
"""
class Navigation(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, blackboard):
    
        super(Navigation, self).__init__(name)
        
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
    
    def setup(self):
        
        # Setup devices and enable them
        
        self.timestep = int(self.robot.getBasicTimeStep())

        self.marker = self.robot.getFromDef("marker").getField("translation")
        self.marker.setSFVec3f([0, 0, 0.5])
        
        self.motorL = self.robot.getDevice("wheel_right_joint")
        self.motorR = self.robot.getDevice("wheel_left_joint")
        self.motorL.setPosition(float('inf'))
        self.motorR.setPosition(float('inf'))
        
        self.MAX_SPEED = 6
        
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)
        
        self.display = self.robot.getDevice("display")
        
        self.logger.debug("  %s [Navigation::setup()]" % self.name)
        
    def initialise(self):
        self.motorR.setVelocity(0)
        self.motorL.setVelocity(0)
       
        self.marker_id = 0
       
        self.WP = self.blackboard.read('waypoints')
        self.blackboard.write("nav_complete", False)
        self.logger.debug("  %s [Navigation::initialise()]" % self.name)
   
    def update(self):
        
        current_marker = [*self.WP[self.marker_id], -0.006]
        self.marker.setSFVec3f(current_marker)
        
        # Get current position (xw, yw) in world frame
        
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        
        # Display current position of robot
        
        self.display.setColor(0x00FFFF)
        self.display.setAlpha(1)
        px, py = world2map(xw, yw)
        self.display.drawPixel(px, py)
        
        # Current orientation of the robot in world frame
        
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        if theta > np.pi:
            theta = theta - 2*np.pi
        
        # Pure distance error
        rho = np.sqrt((current_marker[0] - xw)**2 + (current_marker[1] - yw)**2)
        
        # Angular direction of marker from robot
        angular_err = np.arctan2((current_marker[1] - yw), (current_marker[0] - xw))
        
        # Error in orientation of robot
        alpha = angular_err - theta
        
        if alpha > np.pi:
            alpha = alpha - 2*np.pi
        
        # PID Control
        
        p1 = 5
        p2 = 3
        
        Lvel = rho*p2 + alpha*p1
        Lvel = max(min(Lvel, self.MAX_SPEED), -self.MAX_SPEED)
        
        Rvel = rho*p2 - alpha*p1
        Rvel = max(min(Rvel, self.MAX_SPEED), -self.MAX_SPEED)
        
        self.motorL.setVelocity(Lvel)
        self.motorR.setVelocity(Rvel)
        
        # Select next waypoint to follow after reaching a waypoint
        
        
        if rho < 0.32:
            print(f"Currently at {self.marker_id + 1} out of {len(self.WP)}")
            if self.marker_id == len(self.WP) - 1:
                self.feedback_message = "Last Waypoint Reached"
                self.blackboard.write("nav_complete",  True)
                print("nav_complete")
                return py_trees.common.Status.SUCCESS
            
            else:
                self.marker_id += 1
                print(self.marker_id)
                return py_trees.common.Status.RUNNING   
 
        else:
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.logger.debug("  %s [Navigation::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
               
               
       
        
        
        