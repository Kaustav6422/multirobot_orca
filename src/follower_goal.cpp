#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Transform.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>

using namespace std ;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void robot_map_tf_callback(const geometry_msgs::Twist::ConstPtr& data)
{
    string robot_movebase = "robot_1/move_base" ;
    MoveBaseClient ac(robot_movebase.c_str(),true); 
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        ROS_INFO("%s",robot_movebase.c_str());
    }
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id    = "robot_1/base_link"             ; 
    goal.target_pose.header.stamp       = ros::Time::now()  ;
    goal.target_pose.pose.position.x    = data->linear.x    ; 
    goal.target_pose.pose.position.y    = data->linear.y    ; 
    goal.target_pose.pose.position.z    = 0.0               ; 
    goal.target_pose.pose.orientation.w = data->linear.z    ;
    ROS_INFO("Sending goal")                                ;
    ac.sendGoal(goal)                                       ;  
    ac.waitForResult()                                      ;
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to it's goal");
    else
        ROS_INFO("The base failed to move to it's goal");  

}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "follower_goal");

    ros::NodeHandle nh ;

    ros::Subscriber robotmaptf_sub =  nh.subscribe("robot_map_tf",1000, robot_map_tf_callback);

    ros::spin() ;

    return 0 ;

} 





