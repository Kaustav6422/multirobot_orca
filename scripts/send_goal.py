import numpy
import roslib
import sys
import rospy
import tf 
from std_msgs.msg import String
from std_msgs.msg import Empty      #empty message for landing
from geometry_msgs.msg import Twist #control movement
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    
    rospy.init_node('send_goal', anonymous=True)
    
    robot_0_pubgoal = rospy.Publisher('robot_0/move_base_simple/goal', PoseStamped, queue_size=5)

    x = 0
    y = 3
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pose.header.frame_id = "/robot_0/base_link"
    pose.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
       robot_0_pubgoal.publish(pose)
       rospy.sleep(0.01)


    """
    try:
        robot_0_pubgoal.publish(pose)
        rospy.on_shutdown(shutdown_callback)
        rospy.sleep(0.01)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    """
    


