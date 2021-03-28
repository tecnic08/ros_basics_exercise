#!/usr/bin/env python3

import rospy
from ros_basics_msgs.msg import SimplePoseStamped, SimpleVelocities, LaserScan, ProximitySensors
from ros_basics_msgs.srv import *
import numpy as np
import time
import math

class PID:
    '''PID Class: PID controller to correct linear and angular speed of Thymio'''
    def __init__(self, cPose, gPose, rightObst = None, leftObst=None, avoid=True, kp = None, ki = None, kd = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integralError = 0
        self.history = 0
        self.windupThreshold = 25
        self.t_now = time.time()
        self.t_last = self.t_now
        self.speed = 0.05
        self.base = 0.1102 #0.0551
    
    def getError(self, cPose, gPose):
        """
        Compute the error
        Args:
        cPose (list): current pose of the robot
        gPose (list): goal pose to reach
        Return: error (int)
        """

        gtheta = math.atan2(gPose[1] - cPose[1], gPose[0] - cPose[0])
        ctheta =  cPose[2]
        error = gtheta - ctheta

        if error < -math.pi:
            error += 2*math.pi 
        elif error > math.pi:
            error -=  2*math.pi

        return error
    
    def PIDoutput(self,cPose,gPose):
        """
        Compute the correction to apply according the given error
        Args:
        cPose (list): current pose of the robot
        gPose (list): goal pose to reach
        Return: output of the PID (int)
        """

        error = self.getError(cPose,gPose)

        self.t_now = time.time()
        dt = self.t_now - self.t_last
        self.integralError += error*dt

        kpE = self.kp*error
        kiE = self.ki*self.integralError
        if kiE > self.windupThreshold:
            kiE = self.windupThreshold
        elif kiE < -self.windupThreshold:
            kiE = -self.windupThreshold
        if dt > 0:
            kdE = self.kd*(error-self.history)/dt

        self.history = error 

        return kpE + kiE + kdE

    def getSpeeds(self,cPose,gPose):
        """
        get the speed according the PID weight (output)
        Args:
        cPose (list): current pose of the robot
        gPose (list): goal pose to reach
        Return: linear and angular speed (list)
        """
        w = self.PIDoutput(cPose,gPose)
        vR = (2 * self.speed + w * self.base) / 2
        vL = (2 * self.speed - w * self.base) / 2
        vLin = (vR+vL)/2

        vAng = (vR-vL)/self.base

        speeds = [vLin, vAng]
        return speeds

    def getSpeedsObstacle(self, rightObst, leftObst, avoid, cPose=None,gPose=None):
        """
        get the speed when the robot encounters an obstacle. Go straight in the direction of the goal position.
        Args:
        avoid (bool): True to only turn, False to only go straight
        cPose (list): current pose of the robot
        gPose (list): goal pose to reach
        Return: linear and angular speed (list)
        """
        if rightObst:
            # turn right
            vL = -self.speed
            vR = self.speed
        elif leftObst:
            # turn left
            vL = self.speed
            vR = -self.speed
        else:
            # go straight
            vL = self.speed
            vR = self.speed

        vLin = (vR+vL)/2
        vAng = (vR-vL)/self.base


        if avoid == True:
            speeds = [0, vAng]
        else: 
            speeds = self.getSpeeds(cPose,gPose)
            speeds[1] = 0
        return speeds

class Robot:

    '''Robot class. Implement the ROS interactions and the robot behaviour'''
    def __init__(self, simulation):

        self.simulation = simulation
        self.avoidCriteriaReal = 2200
        self.avoidCriteriaSim = 0.025

        # Init Node
        rospy.init_node('thymio_control_pnode', anonymous=True)
        
        # Velocity Publisher
        self.vpub = rospy.Publisher('set_velocities', SimpleVelocities, queue_size = 10)
        
        # Pose Subscriber
        self.psub = rospy.Subscriber('robot_pose', SimplePoseStamped, self.getPose)

        # Sensor Subscriber /thymio_laser/sensor_0
        if self.simulation:
            self.ssub0 = rospy.Subscriber('thymio_laser/sensor_0', LaserScan, self.sensorCallback)
            self.ssub1 = rospy.Subscriber('thymio_laser/sensor_1', LaserScan, self.sensorCallback)
            self.ssub2 = rospy.Subscriber('thymio_laser/sensor_2', LaserScan, self.sensorCallback)
            self.ssub3 = rospy.Subscriber('thymio_laser/sensor_3', LaserScan, self.sensorCallback)
            self.ssub4 = rospy.Subscriber('thymio_laser/sensor_4', LaserScan, self.sensorCallback)
            self.ssub5 = rospy.Subscriber('thymio_laser/sensor_5', LaserScan, self.sensorCallback)
            self.ssub6 = rospy.Subscriber('thymio_laser/sensor_6', LaserScan, self.sensorCallback)
            self.sensorInt = []

        else:
            self.ssub = rospy.Subscriber('proximity_sensors', ProximitySensors, self.sensorCallback)
            self.sensorInt = ProximitySensors()


        # Init pose type
        self.pose = SimplePoseStamped()

        # Init Goal Pose
        self.goalPose = None

        # Init State of Waypoint List
        self.empty = True

        # Init current and goal XY Pose Lists
        self.cPose = []
        self.gPose = []

        # Avoidance Parameter
        self.avoidanceMode = False
        self.avoidCounter = 0
        self.rightObst = False
        self.leftObst = False
        

    def sensorCallback(self, data):
        """
        Callback function to add the sensor values from the subscriber thymio_laser (simulation) or proximity_sensors (real)
        to the list psValues.
        Args:
        data (list): list of proximity sensor values
        """

        if self.simulation:
            if len(self.sensorInt) < 7:
                self.sensorInt.append(data.ranges[0])
            else:
                self.psValues = self.sensorInt
                self.sensorInt = []
                self.sensorInt.append(data.ranges[0])
        else:
            self.psValues = data.values

    def updateObstacles(self):
        """
        Detect obstacles in Simulation or Real mode according a threshold criteria.
        Store the values in leftObst and rightObst booleans.
        """
        if self.simulation:
            self.leftObst = (self.psValues[0] < self.avoidCriteriaSim) or (self.psValues[1] < self.avoidCriteriaSim)
            self.rightObst = (self.psValues[3] < self.avoidCriteriaSim) or (self.psValues[4] < self.avoidCriteriaSim)
        else:
            self.leftObst = (self.psValues[0] > self.avoidCriteriaReal) or (self.psValues[1] > self.avoidCriteriaReal)
            self.rightObst = (self.psValues[3] > self.avoidCriteriaReal) or (self.psValues[4] > self.avoidCriteriaReal)

    def getPose(self, data):
        """
        Callback function to get the pose of the robot from the robot_pose subscriber.
        """
        # Current X,Y,Z Position
        self.pose.pose.xyz.x = data.pose.xyz.x
        self.pose.pose.xyz.y = data.pose.xyz.y
        self.pose.pose.xyz.z = data.pose.xyz.z

        # Current Orientation
        self.pose.pose.rpy.roll = data.pose.rpy.roll
        self.pose.pose.rpy.pitch = data.pose.rpy.pitch
        self.pose.pose.rpy.yaw = data.pose.rpy.yaw

    def checkArrival(self):
        """
        Check if the robot reach the waypoint from the CheckWaypointReached service.
        Store the state in the boolean arrived
        """
        try:
            # Calling service Proxy
            wp_respond_check = rospy.ServiceProxy('check_waypoint_reached', CheckWaypointReached)
            check = wp_respond_check(pose=self.pose, verbose=False)
            # Checking if we arrived to waypoint
            self.arrived = check.reached
        except rospy.ServiceException as e:
            # Log error otherwise
            rospy.logerr("Failed Call, Cannot check for arrival: %s"%e)

    def getWaypoint(self):
        """
        get the waypoint from the CurrentWaypoint service.
        Store in goalPose (list)
        """
        try:
            # Calling service Proxy
            wp_respond = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
            resp = wp_respond() 
            # Getting Waypoint and state of waypoint list
            self.goalPose = resp.goal
            self.empty = resp.is_empty 
            
        except rospy.ServiceException as e:
            # Log error otherwise
            rospy.logerr("Failed Call, Cannot get current waypoint: %s"%e)

    def moveToGoal(self):
        """
        Move the robot to a waypoint and avoid obstacles.
        """
        # Getting Current Pose
        cx = self.pose.pose.xyz.x
        cy = self.pose.pose.xyz.y
        c_theta = self.pose.pose.rpy.yaw
        self.cPose = [cx, cy, c_theta]

        # Getting Goal Pose
        self.getWaypoint()
        gx = self.goalPose.x
        gy = self.goalPose.y
        self.gPose = [gx, gy]

        #Creating an instance of the PID class
        self.controller = PID(cPose=self.cPose, gPose=self.gPose, kp=2, ki=0, kd=0)

        # Init velocity message type
        vel_msg = SimpleVelocities()

        # Check Arrival at Waypoint
        self.checkArrival()

        # If Waypoint list is empty
        if self.empty == True:
            vel_msg.v = 0
            vel_msg.w = 0
            self.vpub.publish(vel_msg)

        else:

            if not self.avoidanceMode:
                self.updateObstacles()

                if self.rightObst or self.leftObst:
                    rospy.logdebug("obstacle: %r and %r", self.leftObst, self.rightObst)
                    self.avoidCounter = 30
                    self.avoidanceMode = True
            #turn 10 times (30-20) and repeat until it sees an obstacle.
            #if it does not see an obstacle anymore, go 20 times in the direction of the waypoint but without turning
            if (self.avoidCounter > 20): 
                speeds = self.controller.getSpeedsObstacle(self.rightObst, self.leftObst, avoid=True)
                self.avoidCounter -= 1

            elif (self.avoidCounter > 0):
                self.avoidanceMode = False
                speeds = self.controller.getSpeedsObstacle(self.rightObst, self.leftObst, False, self.cPose, self.gPose)
                self.avoidCounter -= 1

            else:
                speeds = self.controller.getSpeeds(self.cPose,self.gPose)

            vel_msg.v = speeds[0]
            vel_msg.w = speeds[1]
            self.vpub.publish(vel_msg)

def main():

    rospy.loginfo("------------- ROS basics with Thymio -------------")

    simulationMode = True
    if simulationMode: rospy.loginfo("Simulation mode")
    else: rospy.loginfo("Real mode")

    rospy.wait_for_service('check_waypoint_reached')
    rospy.wait_for_service('current_waypoint')

    robot = Robot(simulation = simulationMode)
    loop_rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        robot.moveToGoal()

if __name__ == '__main__':
    main()