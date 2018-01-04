#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>

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

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

using namespace std ;

/*
#include "IPlanarGridMap.h"
#include "PlanarGridBinaryMap.h"
#include "PlanarGridContainer.h"
#include "PlanarGridIndex.h"
#include "PlanarGridOccupancyMap.h"private: 
*/

#define MAX_LINEAR_VEL 0.5
#define MAX_ANGULAR_VEL 0.5 

class formation_control
{
public:
	formation_control()    ;
	void spin()            ;
	void update()          ;
	int signum(double val) ;
	void update()          ;	
private: 
	ros::NodeHandle nh ;
    ros::Publisher cmd_vel_pub ;
    ros::Publisher debug_pub ;
    ros::Subscriber position_share_sub ;
	void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data) ;
	void init_variables(); 

	double MIN_DIST ;
	double GOAL_YAW ;

	double robot0_x ;
    double robot0_y ;
    double robot0_xdot  ;
    double robot0_ydot  ;
    double robot0_xdot_prev ;
    double robot0_ydot_prev ;
    double robot0_xdotdot ;
    double robot0_ydotdot ;
    double robot0_yaw ;
    double robot0_vx  ;
    double robot0_omega ;

    double robot1_x ;
    double robot1_y ;
    double robot1_xdot ;
    double robot1_ydot ;
    double robot1_xdotdot ;
    double robot1_ydotdot ;
    double robot1_yaw ;
    double robot1_vx ;
    double robot1_wx ;

    double error_dist_prev ;
    double error_dist ;
    double error_dist_rate ;
    double error_yaw ;
    double error_yaw_rate ;

    double goal_x ;
    double goal_y ;
    double goal_yaw ;

    double g ;
    double omega_n ;
    double k1,k2,k3 ;
    double e1,e2,e3 ;
    double vt1,vt2 ;

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
};

formation_control::formation_control()
{
    // Publish
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",10);
    debug_pub   = nh.advertise<geometry_msgs::Twist>("debug",10);

    // Subscribe
    position_share_sub = nh.subscribe("position_share", 100, &formation_control::formation_callback, this);
}

void formation_control::init_variables()
{
	rate = 20 ;

	MIN_DIST = 1 ;
	GOAL_YAW = 0.785 ;

	robot0_x = 0 ;
    robot0_y = 0 ;
    robot0_xdot = 0 ;
    robot0_ydot = 0 ;
    robot0_xdot_prev = 0 ;
    robot0_ydot_prev = 0 ;
    robot0_xdotdot = 0;
    robot0_ydotdot = 0;
    robot0_yaw = 0;
    robot0_vx  = 0;
    robot0_omega = 0 ; // angular velocity curvature

    robot1_x = 0 ;
    robot1_y = 0 ;
    robot1_xdot = 0 ;
    robot1_ydot = 0 ;
    robot1_xdotdot = 0 ;
    robot1_ydotdot = 0 ;
    robot1_yaw = 0 ;
    robot1_vx = 0 ;
    robot1_wx = 0 ;

    error_dist_prev = 0;
    error_dist = 0 ;
    error_dist_rate = 0 ;
    error_yaw = 0 ;
    error_yaw_rate = 0 ;

    goal_x = 0 ;
    goal_y = 0 ;
    goal_yaw = 0 ;

    g = 10 ;
    omega_n = 0 ;
    k1 = 1 ;
    k2 = 1 ;
    k3 = 1 ;
    e1 = 0 ;
    e2 = 0 ;
    e3 = 0 ;
    vt1 = 0 ;
    vt2 = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
}

int formation_control::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

void formation_control::formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data)
{
	if (data->robot_id == "robot_0")
	{
		robot0_x  = data->pose.pose.position.x ;
        robot0_y  = data->pose.pose.position.y ;
        tf::Quaternion q_robot0(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
        tf::Matrix3x3 m_robot0(q_robot0);
        double roll, pitch, yaw;
        m_robot0.getRPY(roll, pitch, yaw);
        robot0_yaw = yaw ;
        robot0_vx = data->twist.twist.linear.x ;
        //robot0_wx = data->twist.twist.angular.z ;
        robot0_xdot = robot0_vx*cos(robot0_yaw) ;
        robot0_ydot = robot0_vx*sin(robot0_yaw) ;
	}

	if (data->robot_id == "robot_1")
	{
		robot1_x = data->pose.pose.position.x ;
        robot1_y = data->pose.pose.position.y ;
        tf::Quaternion q_robot1(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
        tf::Matrix3x3 m_robot1(q_robot1);
        double roll, pitch, yaw;
        m_robot1.getRPY(roll, pitch, yaw);
        robot1_yaw = yaw ;
        robot1_vx = data->twist.twist.linear.x ;
        //robot1_wx = data->twist.twist.angular.z ;
        robot1_xdot = robot1_vx*cos(robot1_yaw) ;
        robot1_ydot = robot1_vx*sin(robot1_yaw) ;
	}
}

void formation_control::update()
{
	ros::Time now = ros::Time::now();

	if ( now > t_next) 
	{
		elapsed = now.toSec() - then.toSec(); 
		ROS_INFO_STREAM("elapsed =" << elapsed);

	    goal_x = robot0_x + MIN_DIST*cos(GOAL_YAW) ;
        goal_y = robot0_y + MIN_DIST*sin(GOAL_YAW) ;

        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/sample_time ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/sample_time ;
        robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

        omega_n           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
        k1                =  5*omega_n ;
        k2                =  5*omega_n ;
        k3                =  g*abs(robot0_vx);

        e1                =  cos(robot1_yaw)*(goal_x-robot1_x) + sin(robot1_yaw)*(goal_y-robot1_y) ; // ok
        e2                = -sin(robot1_yaw)*(goal_x-robot1_x) + cos(robot1_yaw)*(goal_y-robot1_y) ; // ok
        e3                =  robot0_yaw - robot1_yaw ; //very small

        vt1               = -k1*e1 ;
        vt2               = -k2*signum(robot0_vx)*e2 - k3*e3 ;

        robot0_xdot_prev = robot0_xdot_prev ;
        robot0_ydot_prev = robot0_ydot_prev ;

        geometry_msgs::Twist vel_msg ; 
        vel_msg.linear.x  = robot0_vx*cos(e3) - vt1 ;
        vel_msg.angular.z = robot0_omega - vt2 ; 
        cmd_vel_pub.publish(vel_msg) ;

        geometry_msgs::Twist debug_msg                        ;
        debug_msg.linear.x =  robot0_vx*cos(e3) - vt1         ;
        debug_msg.linear.y =  robot0_omega - vt2              ;
        debug_msg.linear.z =  robot0_yaw                      ;
        debug_msg.angular.x = e2                              ;
        debug_msg.angular.y = e3                              ;
        debug_msg.angular.z = 0                               ;
        debug_pub.publish(debug_msg)                          ;
	}
	else { ; } 
}

void formation_control::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
	{
	   update();
	   loop_rate.sleep();
	}
}  
  

int main(int argc, char **argv)
{ 
  //Initiate ROS
  ros::init(argc, argv, "formation_control");

  // Create an object of class formation_control that will take care of everything
  formation_control formation_one ;

  formation_one.spin() ;
  /*
  ros::spin() ;
  double YOUR_DESIRED_RATE = 20 ;
  while(true) 
  { 
    ros::Rate(YOUR_DESIRED_RATE).sleep() ; 
    ros::spinOnce(); 
  }
  */

  return 0 ;
}
