#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
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

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
      
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_map_tf");
    ros::NodeHandle nh                   ; 

    //char *robot_id = argv[1] ;
    //string robot_name = robot_id ;
    //robot_name += "/move_base" ; ///move_base_simple

    
    //MoveBaseClient ac(robot_name.c_str(),true); 
    //while(!ac.waitForServer(ros::Duration(5.0)))
    //{
    //  ROS_INFO("Waiting for the move_base action server to come up");
    //  ROS_INFO("%s",robot_name.c_str());
    //}
    //move_base_msgs::MoveBaseGoal goal;
    

    double robot0_x ;
    double robot0_y ;
    double robot0_theta ;

    //double robot1_x ;
    //double robot1_y ;
    //double robot1_theta ;
    
    ros::Publisher robotmaptf_pub = nh.advertise<geometry_msgs::Twist>("robot_map_tf", 10);
   
    tf::TransformListener listener_robot0 ;
    //tf::TransformListener listener_robot1 ;

    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);

    listener_robot0.waitForTransform("robot_0/base_link", "map", ros::Time(0), ros::Duration(10.0));
    //listener_robot1.waitForTransform("robot_1/base_link", "map", ros::Time(0), ros::Duration(10.0));

    ros::Rate loopRate(20);

    while (ros::ok())
    {
        /*
        geometry_msgs::TransformStamped transformStamped;
        try
        {
         transformStamped = tfBuffer.lookupTransform("robot_0/base_link", "map",   ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
        }
        */

        tf::StampedTransform transform_robot0 ;
        //tf::StampedTransform transform_robot1 ;
 
        try
        {
            listener_robot0.lookupTransform("robot_0/base_footprint", "/map", ros::Time(0), transform_robot0);
        }
        catch (tf::TransformException ex1)
        {
            ROS_ERROR("%s",ex1.what());
            ros::Duration(1.0).sleep();
        }
        
        /*
        try
        {
            listener_robot1.lookupTransform("robot_1/base_link", "map", ros::Time(0), transform_robot1);
        }
        catch (tf::TransformException ex2)
        {
            ROS_ERROR("%s",ex2.what());
            ros::Duration(1.0).sleep();
        }
        */

        robot0_x     = transform_robot0.getOrigin().x() ;
        robot0_y     = transform_robot0.getOrigin().y() ;
        robot0_theta = getYaw(transform_robot0.getRotation()) ;

        //robot1_x     = transform_robot1.getOrigin().x() ;
        //robot1_y     = transform_robot1.getOrigin().x() ;
        //robot1_theta = getYaw(transform_robot1.getRotation()) ;
   
        /*
        goal.target_pose.header.frame_id = "map"         ; 
        goal.target_pose.header.stamp = ros::Time::now() ;
        goal.target_pose.pose.position.x = goal_x        ; 
        goal.target_pose.pose.position.y = goal_y        ; 
        goal.target_pose.pose.position.z = 0.0           ; 
        goal.target_pose.pose.orientation.w = goal_theta ;
        ROS_INFO("Sending goal") ;
        ac.sendGoal(goal)        ;  
        ac.waitForResult()       ;
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved to it's goal");
        else
            ROS_INFO("The base failed to move to it's goal");
        */

        geometry_msgs::Twist robotmaptf_msg     ;

        robotmaptf_msg.linear.x  = robot0_x     ;
        robotmaptf_msg.linear.y  = robot0_y     ;
        robotmaptf_msg.linear.z  = robot0_theta ;

        //debug_msg.angular.x = robot1_x     ;
        //debug_msg.angular.y = robot1_y     ;
        //debug_msg.angular.z = robot1_theta ;

        robotmaptf_pub.publish(robotmaptf_msg)       ;

        loopRate.sleep() ;
    }
  
    return 0;
}

