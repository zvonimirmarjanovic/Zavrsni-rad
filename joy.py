#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Joy





class Follow():

    def __init__(self):

        self.joystick = Joy()
        self.vel = Twist()
        rospy.Subscriber("/joy", Joy, self.callback1)
        self.pub = rospy.Publisher("/bravo/cmd_vel", Twist, queue_size = 1)

    def callback1(self,data):
        
        print 'x:'+str(data.axes[1]) #kada se joystick naginje naprijed i natrag
        print 'zakret'+str(data.axes[0]) #kada se naginje lijevo ili desno; maxLijevo je 1 maxDesno je -1
        if not data.buttons[7]:
            self.vel.linear.x = max(data.axes[1],0.6)
            self.vel.angular.z = data.axes[0]
        else: 
            self.vel.linear.x = 0
            self.vel.angular.z = 0
        
        self.pub.publish(self.vel)
        
        if data.buttons[8]:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.pub.publish(self.vel)
            rospy.signal_shutdown('Quit') 
        
        
    #TODO dodati za zaustavljanje!!!
        

if __name__=='__main__':

    rospy.init_node('nodeJoy')
    try:
        follow = Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
