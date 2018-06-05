#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
import math
from math import pow,atan2,sqrt,sin,cos
from collections import deque
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

pi = math.pi


class Follow():

    def __init__(self):

        ''' Initialization of parameters; velocities, positions, laser, joystick, subscribers 
        and publishers '''

        self.vel = Twist()
        
        self.tagPose = Pose()
        self.vCalc = deque() 
        self.LCalc = deque()
              
        self.dist = LaserScan()
        self.button = Joy()
        self.buttonNine = False

        self.firstPoseCallback = False
        self.firstLaserCallback = False
        
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        rospy.Subscriber("/echo/scan", LaserScan ,self.laserCallback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.poseCallback)
        
        self.pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size = 1)
        

    def quat_to_angle(self, data):

        ''' Transforming quaternions into Euler angles '''

        q = data.pose.pose.orientation
        angle = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) #angle in radians
        return angle * 360 / (2 * pi)


    def poseCallback(self,data):

        ' Collecting the second robots position '

        if not self.firstPoseCallback: self.firstPoseCallback = True 
        marker = data.markers[0]
        self.tagPose =  marker.pose.pose

    def joyCallback(self,data):

        ' Safety button implementation '
        
        if data.buttons[8]:
            self.buttonNine = True
        else:
            self.buttonNine = False
        
    def laserCallback(self,data):

        ' Collecting data from the laser sensor '
        
        if not self.firstLaserCallback: self.firstLaserCallback = True
        self.dist = data
        self.minDistList = self.dist.ranges[260:280]
        

    def velocityCalc(self,linearVelocity,distance):
        
        '''
        Predicting the velocity with a second degree function with coefficients calculated 
        with polyfit  
        '''
        
        if len(self.vCalc) > 3:
            self.vCalc.pop()
        self.vCalc.appendleft(linearVelocity)
        if len(self.LCalc) > 3:
            self.LCalc.pop()
        self.LCalc.appendleft(distance)
        coeffs = np.polyfit(self.LCalc,self.vCalc,2)
        currentVelocitiy = coeffs[0]*distance**2 + coeffs[1]*distance + coeffs[2]
        return currentVelocitiy
    
    
    def pure_pursuite(self):

        ' Implementation of the pure pursuit algorithm for this case '
   
        x2 = -self.tagPose.position.x
        y2 = self.tagPose.position.y
        
        euclidDist = (self.tagPose.position.x**2 + self.tagPose.position.y**2 
                + self.tagPose.position.z**2)**0.5

        self.noSmallValues = []
            
        for i in range (len(self.minDistList)):
            if self.minDistList[i] > 0.1:
                self.noSmallValues.append(self.minDistList[i])
        
        distance = min(self.noSmallValues)
        #print 'distance s lasera: '+str(distance)

        if self.buttonNine:
            
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.pub.publish(self.vel)
            rospy.signal_shutdown('Quit')
        
        if  distance > 0.7:     
            
            print 'y2 = '+str(y2)
            print 'x2 = '+str(x2)
            print '###############'
            

            angular =  2.0*x2 / (euclidDist**2)
            self.vel.angular.z = angular 

            self.vel.linear.x = min(abs(self.velocityCalc(euclidDist/6, euclidDist)-5*(0.7 - euclidDist)),0.6)
            
            print 'self.vel.angular.z'+str(self.vel.angular.z)
            print 'self.vel.linear.x'+str(self.vel.linear.x)
            
            self.pub.publish(self.vel)
        
        if distance < 0.7:     
            
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.pub.publish(self.vel)


    def run(self):

        ' The run function which runs the algorithm'

        while not rospy.is_shutdown():
            if not self.firstLaserCallback or not self.firstPoseCallback: continue
            self.pure_pursuite()


if __name__ == '__main__':

    rospy.init_node('nodeMovement')
    try:
        follow = Follow()
        follow.run()
    except rospy.ROSInterruptException:
        pass