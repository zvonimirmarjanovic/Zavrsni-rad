#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from math import pow,atan2,sqrt,sin,cos
from nav_msgs.msg import Odometry


pi = math.pi



class Follow():



    def __init__(self):

        ''' Initialization of parameters; velocities, positions, subscribers and publishers '''

        self.vel1 = Twist()
        self.vel2 = Twist()
        self.own_pose1 = Odometry()
        self.own_pose2 = Odometry()
        rospy.Subscriber("/r1/odom", Odometry, self.callback1)
        rospy.Subscriber("/r2/odom", Odometry, self.callback2)
        self.pub1 = rospy.Publisher("/r1/cmd_vel", Twist, queue_size = 1)
        self.pub2 = rospy.Publisher("/r2/cmd_vel", Twist, queue_size = 1)

    def quat_to_angle(self, data):

        ''' Transforming quaternions into Euler angles '''

        q = data.pose.pose.orientation
        angle = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) #kut u radijanima
        return angle * 360 / (2 * pi)

    def callback1(self,data):

        ' Collecting the first robots position '

        self.own_pose1 = data

    def callback2(self,data):

        ' Collecting the second robots position '

        self.own_pose2 = data

    def tf(self):

        ' Setting the second robots position into the first robots coordinate system '

        fi = self.quat_to_angle(self.own_pose1) * (2 * pi / 360) * (-1)  # neki -1
        tmp_x = self.own_pose2.pose.pose.position.x - self.own_pose1.pose.pose.position.x
        tmp_y = self.own_pose2.pose.pose.position.y - self.own_pose1.pose.pose.position.y
        x = tmp_x * cos(fi) - tmp_y * sin(fi)
        y = tmp_x * sin(fi) + tmp_y * cos(fi)
        theta = self.quat_to_angle(self.own_pose2) - self.quat_to_angle(self.own_pose1)

        return x, y, theta


    def pure_pursuite(self):

        ' Implementation of the pure pursuit algorithm for this case '

        x2, y2, theta2 = self.tf()
        L = ((self.own_pose2.pose.pose.position.y - self.own_pose1.pose.pose.position.y) ** 2 + (self.own_pose2.pose.pose.position.x - self.own_pose1.pose.pose.position.x) ** 2) ** 0.5  # udaljenost zasad

        if (abs(x2 - 0.415) > 0.3 or abs(y2) > 0.3) and L > 0:

            self.vel1.angular.z =  2.0*y2 / L
            rospy.loginfo(L)
            self.vel1.linear.x = min(L,0.6) # 1 is the predetermined maximum speed
            self.pub1.publish(self.vel1)

        else:

            self.vel1.angular.z = 0
            self.vel1.linear.x = 0
            self.pub1.publish(self.vel1)





    def run(self):

        ' The run function which runs the algorithm'

        while not rospy.is_shutdown():
            self.pure_pursuite()


if __name__=='__main__':

    rospy.init_node('node')
    try:
        follow = Follow()
        follow.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
