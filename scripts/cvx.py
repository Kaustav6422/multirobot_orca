#!/usr/bin/env python

from cvxopt import matrix, solvers
import numpy
import roslib
import sys
import rospy
import tf 
from std_msgs.msg import String
from std_msgs.msg import Empty      #empty message for landing
from geometry_msgs.msg import Twist #control movement
from geometry_msgs.msg import PoseStamped
from collvoid_msgs.msg import PoseTwistWithCovariance



class formation_control(object):

    def __init__(self):

        #rospy.init_node('formation_control') 
 
        # Publish
        self.cmd_vel_pub = rospy.Publisher('/robot_1/mobile_base/commands/velocity', Twist, queue_size=100, latch=True) 
        self.debug_pub   = rospy.Publisher('debug', Twist, queue_size=100, latch=True)
 
        # Subscribe
        rospy.Subscriber("position_share",PoseTwistWithCovariance, self.formation_callback)

        self.robot0_x   = 0 
        self.robot0_y   = 0
        self.robot0_yaw = 0
        self.robot0_vx  = 0 
        self.robot0_wx  = 0

        self.robot1_x   = 0
        self.robot1_y   = 0
        self.robot1_yaw = 0
        self.robot1_vx  = 0
        self.robot1_wx  = 0 

    def formation_callback(self,data):

        if data.robot_id == "robot_0" :

            self.robot0_x  = data.position.x 
            self.robot0_y  = data.position.y
            self.robot0_vx = data.linear.x 
            self.robot0_wx = data.angular.z 

        elif data.robot_id == "robot_1" :

            self.robot1_x  = data.position.x 
            self.robot1_y  = data.position.y 
            self.robot1_vx = data.linear.x
            self.robot1_wx = data.angular.z 

    def update(self): 
        cmd_msg = Twist() 
        cmd_msg.linear.x = 1 
        cmd_msg.angular.z = 0
        self.cmd_vel_pub.publish(cmd_msg)

    def solve_optimization(self): 
        A = matrix([ [-1.0, -1.0, 0.0, 1.0], [1.0, -1.0, -1.0, -2.0] ])
        b = matrix([ 1.0, -2.0, 0.0, 4.0 ])
        c = matrix([ 2.0, 1.0 ])
        sol=solvers.lp(c,A,b)   

    def spin(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update() 
            self.solve_optimization()
            r.sleep()
        
if __name__ == "__main__":
    
    rospy.init_node('cvx', anonymous=True)
    formation = formation_control()
    formation.spin()
    #rospy.spin()
    
