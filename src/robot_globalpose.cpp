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
#include "collvoid_msgs/PoseTwistWithCovariance.h"

using namespace std ;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goal_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data)
{
    string robot_movebase = "robot_1/move_base"                             ;
    MoveBaseClient ac(robot_movebase.c_str(),true)                          ; 
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up")      ;
        ROS_INFO("%s",robot_movebase.c_str())                               ;
    }
    move_base_msgs::MoveBaseGoal goal                                       ;

    if (data->robot_id == "robot_0") 
    {
        goal.target_pose.header.frame_id    = "map"           ; 
        goal.target_pose.header.stamp       = ros::Time::now()              ;
        goal.target_pose.pose.position.x    = data->pose.pose.position.x    ; 
        goal.target_pose.pose.position.y    = data->pose.pose.position.y    ; 
        goal.target_pose.pose.position.z    = 0.0                           ; 
        goal.target_pose.pose.orientation   = data->pose.pose.orientation   ;
        ROS_INFO("Sending goal")                                            ;
        ac.sendGoal(goal)                                                   ;  
        ac.waitForResult()                                                  ;
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved to it's goal")                 ;
        else
            ROS_INFO("The base failed to move to it's goal")                ;
    }  
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "robot_gloalpose");

    ros::NodeHandle nh ;

    ros::Subscriber robotmaptf_sub =  nh.subscribe("position_share", 10, goal_callback);

    ros::spin() ;

    return 0 ;

} 

