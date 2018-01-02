#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Transform.h"
#include <tf/tf.h>
#include <string>

using namespace std ;
   
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
   
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "send_goal_follower1");
    ros::NodeHandle nh("~") ;

    char *robot_id = argv[1] ;
    string robot_name = robot_id ;
    robot_name += "/move_base" ;
   
    double goal_x ;
    double goal_y ;
    double goal_theta ;
    if (!nh.getParam("goal_x", goal_x)) 
       goal_x = 0 ;
    if (!nh.getParam("goal_y", goal_y))
       goal_y = 0 ;
    if (!nh.getParam("goal_theta", goal_theta))
       goal_theta = 0 ;
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(robot_name.c_str(),true); //(robot_name.c_str(), true); 
   
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
      ROS_INFO("%s",robot_name.c_str());
    }
   
    move_base_msgs::MoveBaseGoal goal;
   
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map" ; // "robot_0/base_link";
    goal.target_pose.header.stamp = ros::Time::now();
   
    goal.target_pose.pose.position.x = goal_x ; //0.0 ;
    goal.target_pose.pose.position.y = goal_y ; //3.0 ;
    goal.target_pose.pose.position.z = 0.0 ; //0.0 ;
    // goal.target_pose.pose.orientation.w = 0.0;

    
    double radians = goal_theta * (M_PI/180) ;
    tf::Quaternion quaternion ;
    quaternion = tf::createQuaternionFromYaw(radians);
    geometry_msgs::Quaternion qMsg ;
    tf::quaternionTFToMsg(quaternion, qMsg) ;
    goal.target_pose.pose.orientation = qMsg ;
    
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
  
    ac.waitForResult();
  
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("Hooray, the base moved to it's goal");
    else
       ROS_INFO("The base failed to move to it's goal");
  
    return 0;
}

