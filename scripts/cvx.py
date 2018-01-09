#!/usr/bin/env python

from cvxopt import matrix, solvers
from cvxopt.modeling import variable, max
from cvxopt.modeling import op, dot

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
        self.robot0_euler = [] 
        self.robot0_X1  = 0 # chained form
        self.robot0_X2  = 0 # chained form
        self.robot0_X3  = 0 # chained form
        self.robot0_U1  = 0 # chained form
        self.robot0_U2  = 0 # chained form
        


        self.robot1_x   = 0
        self.robot1_y   = 0
        self.robot1_yaw = 0
        self.robot1_vx  = 0
        self.robot1_wx  = 0
        self.robot1_euler = []
        self.robot1_X1  = 0 # chained form
        self.robot1_X2  = 0 # chained form
        self.robot1_X3  = 0 # chained form
        self.robot1_U1  = 0 # chained form
        self.robot1_U2  = 0 # chained form

    def formation_callback(self,data):

        if data.robot_id == "robot_0" :

            self.robot0_x  = data.position.x 
            self.robot0_y  = data.position.y
            self.robot0_vx = data.linear.x 
            self.robot0_wx = data.angular.z 
            self.robot0_euler = tf.transformations.euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
            self.robot0_yaw = self.robot0_euler[2]

            # Chained form
            self.robot0_X1 = self.robot0_yaw 
            self.robot0_X2 = self.robot0_x*cos(self.robot0_yaw) + self.robot0_y*sin(self.robot0_yaw)
            self.robot0_X3 = self.robot0_x*sin(self.robot0_yaw) - self.robot0_y*cos(self.robot0_yaw)
            self.robot0_U1 = self.robot0_wx 
            self.robot0_U2 = self.robot0_vx - self.robot0_X3*self.robot0_U1 

        elif data.robot_id == "robot_1" :

            self.robot1_x  = data.position.x 
            self.robot1_y  = data.position.y 
            self.robot1_vx = data.linear.x
            self.robot1_wx = data.angular.z 
            self.robot1_euler = tf.transformations.euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
            self.robot1_yaw = self.robot1_euler[2] 

            # Chained form
            self.robot1_X1 = self.robot1_yaw 
            self.robot1_X2 = self.robot1_x*cos(self.robot1_yaw) + self.robot1_y*sin(self.robot1_yaw)
            self.robot1_X3 = self.robot1_x*sin(self.robot1_yaw) - self.robot1_y*cos(self.robot1_yaw)
            self.robot1_U1 = self.robot1_wx 
            self.robot1_U2 = self.robot1_vx - self.robot1_X3*self.robot1_U1 

    def update(self): 
        cmd_msg = Twist() 
        cmd_msg.linear.x = 1 
        cmd_msg.angular.z = 0
        self.cmd_vel_pub.publish(cmd_msg)

    def solve_optimization(self): 


        #x = variable(10, 'x')
        #g = max(abs(x)) # infinity norm

        #x = variable()
        #y = variable()
        #c1 = ( 2*x+y <= 3 )
        #c2 = ( x+2*y <= 3 )
        #c3 = ( x >= 0 )
        #c4 = ( y >= 0 )
        #lp1 = op(-4*x-5*y, [c1,c2,c3,c4])
        #lp1.solve()
        #lp1.status

        #x = variable(2)
        #A = matrix([[2.,1.,-1.,0.], [1.,2.,0.,-1.]])
        #b = matrix([3.,3.,0.,0.])
        #c = matrix([-4.,-5.])
        #ineq = ( A*x <= b )
        #lp2 = op(dot(c,x), ineq)
        #lp2.solve()
        #print(lp2.objective.value())
        #print(x.value)

        #m, n = 500, 100
        #A = normal(m,n)
        #b = normal(m)
        #x1 = variable(n)
        #op(max(abs(A*x1-b))).solve() # ||Ax-b||_{inf}



        c = matrix([-6., -4., -5.])
        G = matrix([[ 16., 7.,  24.,  -8.,   8.,  -1.,  0., -1.,  0.,  0.,
                   7.,  -5.,   1.,  -5.,   1.,  -7.,   1.,   -7.,  -4.],
                [-14., 2.,   7., -13., -18.,   3.,  0.,  0., -1.,  0.,
                   3.,  13.,  -6.,  13.,  12., -10.,  -6.,  -10., -28.],
                [  5., 0., -15.,  12.,  -6.,  17.,  0.,  0.,  0., -1.,
                   9.,   6.,  -6.,   6.,  -7.,  -7.,  -6.,   -7., -11.]])
        h = matrix( [ -3., 5.,  12.,  -2., -14., -13., 10.,  0.,  0.,  0.,
                  68., -30., -19., -30.,  99.,  23., -19.,   23.,  10.] )
        dims = {'l': 2, 'q': [4, 4], 's': [3]}
        sol = solvers.conelp(c, G, h, dims)

    def spin(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update() 
            now = rospy.get_rostime()
            rospy.loginfo("Now time : %i ", now.secs)
            self.solve_optimization()
            later = rospy.get_rostime()
            rospy.loginfo("Later time : %i ", later.secs)

            r.sleep()
        
if __name__ == "__main__":
    
    rospy.init_node('cvx', anonymous=True)
    formation = formation_control()
    formation.spin()
    
