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
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Joy.h>

#include <fstream>
#include <sstream>
#include <iostream>



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
	int signum(double val) ;
private: 
	ros::NodeHandle nh ;

    ros::Publisher cmd_vel_pub ;
    ros::Publisher debug_pub ;
    ros::Publisher cmd_vel_leader_pub ;
    ros::Publisher formation_positionshare_pub ;

    ros::Subscriber position_share_sub ;
    ros::Subscriber odom_leader_sub ;
    ros::Subscriber odom_robot1_sub ;
    ros::Subscriber joy_sub ;

	void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data) ;
    void odom_callback_leader(const nav_msgs::Odometry::ConstPtr& msg) ;
	void odom_callback_robot1(const nav_msgs::Odometry::ConstPtr& msg) ;
	void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy) ;
	void init_variables()  ;

	void update()          ;
	void write_robot1_data(std::string filename) ;

	geometry_msgs::Twist debug_msg      ;
	geometry_msgs::Twist vel_msg        ; 

	tf::TransformListener tf_ ;

	double SWARM_DIST ;
	double SWARM_ANG ;
	double FORMATION_ANG ;

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
    double robot0_wx  ;
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
    double k1,k2,k3, k1_gain, k2_gain ;
    double e1,e2,e3 ;
    double vt1,vt2 ;

    double k_s ;
    double k_theta ;
    double k_orientation ; 
    double e_s ;
    double e_theta ;
    double e_orientation ;
    double theta ;
    double delta_l ;

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
    ros::Time then;
};

formation_control::formation_control()
{
	init_variables();

    // Publish
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",20);
    cmd_vel_leader_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/mobile_base/commands/velocity",20);
    debug_pub   = nh.advertise<geometry_msgs::Twist>("debug",20);
    formation_positionshare_pub = nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("formation_positionshare", 20) ;

    // Subscribe
    //position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
    joy_sub         = nh.subscribe("robot_0/joy",20,&formation_control::joystick_callback,this) ;
    odom_leader_sub = nh.subscribe("/robot_0/odom", 20, &formation_control::odom_callback_leader, this) ;
    odom_robot1_sub = nh.subscribe("/robot_1/odom", 20, &formation_control::odom_callback_robot1, this) ;
}

void formation_control::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist leader_vel_msg ;
    double joystick_vd = joy->axes[1] ;
    double joystick_wd = joy->axes[2] ;
    leader_vel_msg.linear.x =  joystick_vd;
    leader_vel_msg.angular.z = joystick_wd;
    cmd_vel_leader_pub.publish(leader_vel_msg);
}

void formation_control::init_variables()
{
	rate = 20 ;

	SWARM_DIST = 1.5 ;
	SWARM_ANG = 0.785 ;
	FORMATION_ANG = 0 ;

	robot0_x = 0 ;
    robot0_y = 0 ;
    robot0_xdot = 0 ;
    robot0_ydot = 0 ;
    robot0_xdot_prev = 0 ;
    robot0_ydot_prev = 0 ;
    robot0_xdotdot = 0;
    robot0_ydotdot = 0;
    robot0_yaw = 0;
    robot0_vx  = 0.05;
    robot0_wx = 0 ; 
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

    g = 5 ;
    omega_n = 0 ;
    k1 = 0 ;
    k2 = 0 ;
    k1_gain = 1 ;
    k2_gain = 1 ;
    k3 = 1 ;
    e1 = 0 ;
    e2 = 0 ;
    e3 = 0 ;
    vt1 = 0 ;
    vt2 = 0 ;

    k_s = 1 ;
    k_theta = 1 ;
    k_orientation = 1 ;
    e_s = 0 ;
    e_theta = 0 ;
    e_orientation = 0;
    theta = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();
}

int formation_control::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}


void formation_control::odom_callback_leader(const nav_msgs::Odometry::ConstPtr& data) 
{
	boost::mutex::scoped_lock(me_lock_);

	collvoid_msgs::PoseTwistWithCovariance me_msg;
	me_msg.header.stamp = ros::Time::now();
    me_msg.header.frame_id = "map";

    tf::Stamped <tf::Pose> global_pose;
    global_pose.setIdentity();
    global_pose.frame_id_ = "robot_0/base_link";
    global_pose.stamp_ = me_msg.header.stamp ;

    try 
    {
        tf_.waitForTransform("map", "robot_0/base_link", global_pose.stamp_, ros::Duration(0.2));
        tf_.transformPose("map", global_pose, global_pose);
    }
    catch (tf::TransformException ex) 
    {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        //return false;
    };

    geometry_msgs::PoseStamped pose_msg;
    tf::poseStampedTFToMsg(global_pose, pose_msg);

    me_msg.pose.pose = pose_msg.pose;
    me_msg.twist.twist.linear.x  = data->twist.twist.linear.x  ;
    me_msg.twist.twist.linear.y  = data->twist.twist.linear.y  ;
    me_msg.twist.twist.angular.z = data->twist.twist.angular.z ;
    me_msg.robot_id              = "robot_0";
    formation_positionshare_pub.publish(me_msg) ;

	robot0_x = pose_msg.pose.position.x ;
	robot0_y = pose_msg.pose.position.y ;
	tf::Quaternion q_robot0(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
    tf::Matrix3x3 m_robot0(q_robot0);
    double roll, pitch, yaw;
    m_robot0.getRPY(roll, pitch, yaw);
    robot0_yaw = yaw ;
    robot0_vx = data->twist.twist.linear.x ;
    robot0_wx = data->twist.twist.angular.z ;
    robot0_xdot = robot0_vx*cos(robot0_yaw) ;
    robot0_ydot = robot0_vx*sin(robot0_yaw) ;
}

void formation_control::odom_callback_robot1(const nav_msgs::Odometry::ConstPtr& data)
{
	boost::mutex::scoped_lock(me_lock_);

	collvoid_msgs::PoseTwistWithCovariance me_msg;
	me_msg.header.stamp = ros::Time::now();
    me_msg.header.frame_id = "map" ;

    tf::Stamped <tf::Pose> global_pose;
    global_pose.setIdentity();
    global_pose.frame_id_ = "robot_1/base_link";
    global_pose.stamp_ = me_msg.header.stamp ;

    try 
    {
        tf_.waitForTransform("map", "robot_1/base_link", global_pose.stamp_, ros::Duration(0.2));
        tf_.transformPose("map", global_pose, global_pose);
    }
    catch (tf::TransformException ex) 
    {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        //return false;
    };

    geometry_msgs::PoseStamped pose_msg;
    tf::poseStampedTFToMsg(global_pose, pose_msg);

    me_msg.pose.pose = pose_msg.pose;
    me_msg.twist.twist.linear.x  = data->twist.twist.linear.x  ;
    me_msg.twist.twist.linear.y  = data->twist.twist.linear.y  ;
    me_msg.twist.twist.angular.z = data->twist.twist.angular.z ;
    me_msg.robot_id              = "robot_1";
    formation_positionshare_pub.publish(me_msg) ;

	robot1_x = pose_msg.pose.position.x ; //data->pose.pose.position.x ;
    robot1_y = pose_msg.pose.position.y ; //data->pose.pose.position.y ;
    //tf::Quaternion q_robot1(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
    tf::Quaternion q_robot1(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
    tf::Matrix3x3 m_robot1(q_robot1);
    double roll, pitch, yaw;
    m_robot1.getRPY(roll, pitch, yaw);
    robot1_yaw = yaw ;
    robot1_vx = data->twist.twist.linear.x ;
    robot1_xdot = robot1_vx*cos(robot1_yaw) ;
    robot1_ydot = robot1_vx*sin(robot1_yaw) ;
}


/*
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
*/

void formation_control::update()
{
	ros::Time now = ros::Time::now();

	if ( now > t_next) 
	{
		elapsed = now.toSec() - then.toSec(); 
		//ROS_INFO_STREAM("elapsed =" << elapsed);
		//ROS_INFO_STREAM("now =" << now.toSec());
		//ROS_INFO_STREAM("then =" << then.toSec());

		FORMATION_ANG = 1.57 - SWARM_ANG - robot0_yaw ;
                // FORMATION_ANG2 = 1.57 + SWARM_ANG - robot0_yaw ;

		if (FORMATION_ANG >= 2*M_PI)
			FORMATION_ANG = FORMATION_ANG - 2*M_PI ;
		if (FORMATION_ANG <= 2*M_PI)
			FORMATION_ANG = FORMATION_ANG + 2*M_PI ;

	    goal_x = robot0_x - SWARM_DIST*sin(FORMATION_ANG) ;
        goal_y = robot0_y - SWARM_DIST*cos(FORMATION_ANG) ;

        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/elapsed ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/elapsed ;

        if (abs(robot0_vx) >= 0.05) 
        {
        	robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

            omega_n           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
            k1                =  k1_gain*omega_n ;
            k2                =  k2_gain*omega_n ;
            k3                =  g*abs(robot0_vx);

            e1                =  cos(robot1_yaw)*(goal_x-robot1_x) + sin(robot1_yaw)*(goal_y-robot1_y) ; // ok
            e2                = -sin(robot1_yaw)*(goal_x-robot1_x) + cos(robot1_yaw)*(goal_y-robot1_y) ; // ok
            e3                =  robot0_yaw - robot1_yaw ; //very small

            vt1               = -k1*e1 ;
            vt2               = -k2*signum(robot0_vx)*e2 - k3*e3 ;

            robot0_xdot_prev = robot0_xdot      ;
            robot0_ydot_prev = robot0_ydot      ;
            
            vel_msg.linear.x  = robot0_vx*cos(e3) - vt1 ;
            vel_msg.angular.z = robot0_omega - vt2      ; 
            cmd_vel_pub.publish(vel_msg) ;
            
            debug_msg.linear.x =  robot0_yaw                ;
            debug_msg.linear.y =  robot1_yaw               ;
            debug_msg.linear.z =  robot0_omega                      ;
            debug_msg.angular.x = robot1_y                              ;
            debug_msg.angular.y = goal_x                             ;
            debug_msg.angular.z = goal_y                               ;
            debug_pub.publish(debug_msg)                          ;
        }
        else 
        {
        	delta_l = sqrt(pow(goal_y-robot1_y,2) + pow(goal_x-robot1_x,2))   ;
        	theta = atan2(goal_y-robot1_y,goal_x-robot1_x)                    ;
        	e_theta = theta - robot1_yaw ;
        	e_orientation = robot0_yaw - robot1_yaw ;
        	e_s = delta_l*cos(e_theta) ;
        	vel_msg.linear.x  = 0 ; //k_s*e_s ;
        	//if (delta_l > 0.5)
        	//{
        		vel_msg.angular.z = 0 ; //k_theta*e_theta ;
        	//}
        	//else
        	//{
        	//	vel_msg.angular.z = 0 ; //k_orientation*e_orientation ;
        	//}
            cmd_vel_pub.publish(vel_msg) ;
        }
        

        then = now;

        ros::spinOnce();
	}
	else 
	{
		ROS_INFO_STREAM("LOOP MISSED");
	} 
    
}

void formation_control::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
	{
	   update();
	   write_robot1_data("/home/kaustav/catkin_ws/src/multirobot_orca/data/robot1_data.txt") ;
	   loop_rate.sleep();
	}
} 

void formation_control::write_robot1_data(std::string filename) 
{
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename.c_str(), std::ios::app);
	dataFile << robot1_x << " " << robot1_y << " " << robot1_yaw << " " << robot1_vx << " " << robot1_wx << "\n";
	dataFile.close();
} 
  

int main(int argc, char **argv)
{ 
  //Initiate ROS
  ros::init(argc, argv, "formation_control");

  // Create an object of class formation_control that will take care of everything
  formation_control formation_one ;
  tf::TransformListener tf;

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
