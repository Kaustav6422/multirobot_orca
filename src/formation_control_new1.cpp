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
    double integral(double eta1, double eta2, double elapsed); 
private: 
	ros::NodeHandle nh ;

    ros::Publisher cmd_vel_pub_robot1 ;
    ros::Publisher cmd_vel_pub_robot2 ;

    ros::Publisher debug_pub ;
    ros::Publisher cmd_vel_leader_pub ;
    ros::Publisher formation_positionshare_pub ;


    ros::Subscriber position_share_sub ;
    ros::Subscriber odom_leader_sub ;
    ros::Subscriber odom_robot1_sub ;
    ros::Subscriber odom_robot2_sub ;
    ros::Subscriber joy_sub ;

	void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data) ;
    void odom_callback_leader(const nav_msgs::Odometry::ConstPtr& msg) ;
	void odom_callback_robot1(const nav_msgs::Odometry::ConstPtr& msg) ;
	void odom_callback_robot2(const nav_msgs::Odometry::ConstPtr& msg) ;
	void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy) ;
	void init_variables()  ;

	void formation_robot1() ;
	void formation_robot2() ;

	void update()          ;
	void write_robot1_data(std::string filename) ;

	geometry_msgs::Twist debug_msg      ;
	geometry_msgs::Twist vel_msg_robot1 ;
	geometry_msgs::Twist vel_msg_robot2 ; 

	tf::TransformListener tf_ ;

	double SWARM_DIST ;
	double SWARM_ANG ;
	double FORMATION_ANG_robot1 ;
	double FORMATION_ANG_robot2 ;

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
    double robot1_vx_prev ;
    double robot1_vx_dyn ;
    double robot1_wx ;

    double robot2_x ;
    double robot2_y ;
    double robot2_xdot ;
    double robot2_ydot ;
    double robot2_xdotdot ;
    double robot2_ydotdot ;
    double robot2_yaw ;
    double robot2_vx ;
    double robot2_vx_prev ;
    double robot2_vx_dyn  ;
    double robot2_wx ;

    double error_dist_prev ;
    double error_dist ;
    double error_dist_rate ;
    double error_yaw ;
    double error_yaw_rate ;

    double goal_x_r1 ;
    double goal_y_r1 ;
    double goal_yaw_r1 ;

    double goal_x_r2 ;
    double goal_y_r2 ;
    double goal_yaw_r2 ;

    // robot1 controller parameters
    double g_r1 ;
    double omega_n_r1 ;
    double k1_r1,k2_r1,k3_r1 ;
    double k1_gain_r1, k2_gain_r1 ;
    double e1_r1,e2_r1,e3_r1 ;
    double vt1_r1,vt2_r1 ;
    double k_s_r1 ;
    double k_theta_r1 ;
    double k_orientation_r1 ; 
    double e_s_r1 ;
    double e_theta_r1 ;
    double e_orientation_r1 ;
    double theta_r1 ;
    double delta_l_r1 ;

    // robot2 controller parameters
    double g_r2 ;
    double omega_n_r2 ;
    double k1_r2 ,k2_r2,k3_r2 ;
    double k1_gain_r2, k2_gain_r2 ;
    double e1_r2,e2_r2,e3_r2 ;
    double vt1_r2,vt2_r2 ;
    double k_s_r2 ;
    double k_theta_r2 ;
    double k_orientation_r2 ; 
    double e_s_r2 ;
    double e_theta_r2 ;
    double e_orientation_r2 ;
    double theta_r2 ;
    double delta_l_r2 ;

    // new controller
    double k1_new,k2_new,k3_new ;

    // dynamic feedback linearization
    double kp1, kd1, kp2, kd2 ;
    double eta_r1, eta_r1_prev, eta_r1_dot, eta_r1_dot_prev ;
    double eta_r2, eta_r2_prev, eta_r2_dot, eta_r2_dot_prev ;

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
    cmd_vel_leader_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/mobile_base/commands/velocity",20);
    cmd_vel_pub_robot1 = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",20);
    cmd_vel_pub_robot2 = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",20);
    debug_pub          = nh.advertise<geometry_msgs::Twist>("debug",20);
    //formation_positionshare_pub = nh.advertise<collvoid_msgs::PoseTwistWithCovariance>("/formation_positionshare", 50) ;

    // Subscribe
    //position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
    //joy_sub            = nh.subscribe("/robot_0/joy",50,&formation_control::joystick_callback,this) ;
    odom_leader_sub      = nh.subscribe("/robot_0/odom", 50, &formation_control::odom_callback_leader, this) ;
    odom_robot1_sub      = nh.subscribe("/robot_1/odom", 50, &formation_control::odom_callback_robot1, this) ;
    odom_robot2_sub      = nh.subscribe("/robot_2/odom", 50, &formation_control::odom_callback_robot2, this) ;
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
	FORMATION_ANG_robot1 = 0 ;
	FORMATION_ANG_robot2 = 0 ;

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
    robot0_wx  = 0 ; 
    robot0_omega = 0 ; // angular velocity curvature

    robot1_x = 0 ;
    robot1_y = 0 ;
    robot1_xdot = 0 ;
    robot1_ydot = 0 ;
    robot1_xdotdot = 0 ;
    robot1_ydotdot = 0 ;
    robot1_yaw = 0 ;
    robot1_vx = 0 ;
    robot1_vx_prev = 0 ;
    robot1_vx_dyn  = 0 ;
    robot1_wx = 0 ;

    robot2_x = 0 ;
    robot2_y = 0 ;
    robot2_xdot = 0 ;
    robot2_ydot = 0 ;
    robot2_xdotdot = 0 ;
    robot2_ydotdot = 0 ;
    robot2_yaw = 0 ;
    robot2_vx = 0 ;
    robot2_vx_prev = 0 ;
    robot2_vx_dyn  = 0 ;
    robot2_wx = 0 ;
    

    error_dist_prev = 0;
    error_dist = 0 ;
    error_dist_rate = 0 ;
    error_yaw = 0 ;
    error_yaw_rate = 0 ;

    goal_x_r1 = 0 ;
    goal_y_r1 = 0 ;
    goal_yaw_r1 = 0 ;

    goal_x_r2 = 0 ;
    goal_y_r2 = 0 ;
    goal_yaw_r2 = 0 ;

    // robot1
    g_r1 = 1 ;
    omega_n_r1 = 0 ;
    k1_r1 = 0 ;
    k2_r1 = 0 ;
    k1_gain_r1 = 5 ;
    k2_gain_r1 = 5 ;
    k3_r1 = 5 ;
    e1_r1 = 0 ;
    e2_r1 = 0 ;
    e3_r1 = 0 ;
    vt1_r1 = 0 ;
    vt2_r1 = 0 ;
    k_s_r1 = 1 ;
    k_theta_r1 = 1 ;
    k_orientation_r1 = 1 ;
    e_s_r1 = 0 ;
    e_theta_r1 = 0 ;
    e_orientation_r1 = 0;
    theta_r1 = 0 ;

    // robot2
    g_r2 = 1 ;
    omega_n_r2 = 0 ;
    k1_r2 = 0 ;
    k2_r2 = 0 ;
    k1_gain_r2 = 5 ;
    k2_gain_r2 = 5 ;
    k3_r2 = 1 ;
    e1_r2 = 0 ;
    e2_r2 = 0 ;
    e3_r2 = 0 ;
    vt1_r2 = 0 ;
    vt2_r2 = 0 ;
    k_s_r2 = 1 ;
    k_theta_r2 = 1 ;
    k_orientation_r2 = 1 ;
    e_s_r2 = 0 ;
    e_theta_r2 = 0 ;
    e_orientation_r2 = 0;
    theta_r2 = 0 ;

    // new controller
    k1_new = 1.5 ;
    k2_new = 4 ;
    k3_new = 2 ;

    // Dynamics feedback linearization
    kp1  = 1 ;
    kd1  = 3 ;
    kp2  = 1 ;
    kd2  = 3 ;
    eta_r1 = 0 ;
    eta_r1_prev = 0 ;
    eta_r1_dot = 0 ;
    eta_r1_dot_prev = 0 ;
    eta_r2 = 0 ;
    eta_r2_prev = 0 ;
    eta_r2_dot = 0 ;
    eta_r2_dot_prev = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();
    // elapsed = t_delta ;
}

int formation_control::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

double formation_control::integral(double eta, double eta_prev, double elapsed)
{
  return (elapsed/2)*(eta + eta_prev) ;
}

void formation_control::odom_callback_leader(const nav_msgs::Odometry::ConstPtr& data) 
{
	// boost::mutex::scoped_lock(me_lock_);

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
	//boost::mutex::scoped_lock(me_lock_);

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

void formation_control::odom_callback_robot2(const nav_msgs::Odometry::ConstPtr& data)
{
	//boost::mutex::scoped_lock(me_lock_);

	collvoid_msgs::PoseTwistWithCovariance me_msg;
	me_msg.header.stamp = ros::Time::now();
    me_msg.header.frame_id = "map" ;

    tf::Stamped <tf::Pose> global_pose;
    global_pose.setIdentity();
    global_pose.frame_id_ = "robot_2/base_link";
    global_pose.stamp_ = me_msg.header.stamp ;

    try 
    {
        tf_.waitForTransform("map", "robot_2/base_link", global_pose.stamp_, ros::Duration(0.2));
        tf_.transformPose("map", global_pose, global_pose);
    }
    catch (tf::TransformException ex) 
    {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        //return false;
    };

    geometry_msgs::PoseStamped pose_msg ;
    tf::poseStampedTFToMsg(global_pose, pose_msg);

    me_msg.pose.pose = pose_msg.pose;
    me_msg.twist.twist.linear.x  = data->twist.twist.linear.x  ;
    me_msg.twist.twist.linear.y  = data->twist.twist.linear.y  ;
    me_msg.twist.twist.angular.z = data->twist.twist.angular.z ;
    me_msg.robot_id              = "robot_2";
    formation_positionshare_pub.publish(me_msg) ;

	robot2_x = pose_msg.pose.position.x ; //data->pose.pose.position.x ;
    robot2_y = pose_msg.pose.position.y ; //data->pose.pose.position.y ;
    //tf::Quaternion q_robot1(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
    tf::Quaternion q_robot2(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
    tf::Matrix3x3 m_robot2(q_robot2);
    double roll, pitch, yaw;
    m_robot2.getRPY(roll, pitch, yaw);
    robot2_yaw  = yaw ;
    robot2_vx   = data->twist.twist.linear.x ;
    robot2_xdot = robot2_vx*cos(robot2_yaw) ;
    robot2_ydot = robot2_vx*sin(robot2_yaw) ;
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

void formation_control::formation_robot1()
{
        //boost::mutex::scoped_lock(me_lock_);
    
	FORMATION_ANG_robot1 = -(M_PI - SWARM_ANG - robot0_yaw) ;

	if (FORMATION_ANG_robot1 >= 2*M_PI)
		FORMATION_ANG_robot1 = FORMATION_ANG_robot1 - 2*M_PI ;
	if (FORMATION_ANG_robot1 <= 2*M_PI)
		FORMATION_ANG_robot1 = FORMATION_ANG_robot1 + 2*M_PI ;

    goal_x_r1 = robot0_x + SWARM_DIST*cos(FORMATION_ANG_robot1) ;
    goal_y_r1 = robot0_y + SWARM_DIST*sin(FORMATION_ANG_robot1) ;

    if (abs(robot0_vx) >= 0.05) 
    {
        
        omega_n_r1           =  sqrt(pow(robot0_omega,2) + g_r1*pow(robot0_vx,2)) ;
        k1_r1                =  2*0.7*omega_n_r1 ;
        k2_r1                =  2*0.7*omega_n_r1 ;
        k3_r1                =  3*abs(robot0_vx);

        e1_r1                =  cos(robot1_yaw)*(goal_x_r1-robot1_x) + sin(robot1_yaw)*(goal_y_r1-robot1_y) ; 
        e2_r1                = -sin(robot1_yaw)*(goal_x_r1-robot1_x) + cos(robot1_yaw)*(goal_y_r1-robot1_y) ; 
        e3_r1                =  robot0_yaw - robot1_yaw ;

        // Controller 1 (Linearized controller)
        // vt1_r1                   = -k1_r1*e1_r1 ;
        // vt2_r1                   = -k3_r1*signum(robot0_vx)*e2_r1 - k2_r1*e3_r1 ;
        // vel_msg_robot1.linear.x  = robot0_vx*cos(e3_r1) - vt1_r1 ;
        // vel_msg_robot1.angular.z = robot0_omega - vt2_r1      ; 

        // Controller 2 
        // vt1_r1               = -k1_new*e1_r1 ;
        // vt2_r1               = -k2_new*robot0_vx*e2_r1 - k3_new*robot0_vx*sin(e3_r1) ;
        // vel_msg_robot1.linear.x  = robot0_vx*cos(e3_r1) - vt1_r1 ;
        // vel_msg_robot1.angular.z = robot0_wx - vt2_r1 ;

        // Controller 3( Lyapunov based controller)
        vt1_r1               = -k1_r1*e1_r1 ;
        vt2_r1               = -3*robot0_vx*(sin(e3_r1)/e3_r1)*e2_r1 - k2_r1*e3_r1 ;
        vel_msg_robot1.linear.x  = robot0_vx*cos(e3_r1) -vt1_r1 ;
        vel_msg_robot1.angular.z = robot0_omega - vt2_r1 ;
     }
    else 
    {
        vel_msg_robot1.linear.x = 0 ;
        vel_msg_robot1.angular.z = 0 ;
    }

    cmd_vel_pub_robot1.publish(vel_msg_robot1) ;

}

void formation_control::formation_robot2()
{

    //boost::mutex::scoped_lock(me_lock_);

	FORMATION_ANG_robot2 = -(M_PI + SWARM_ANG - robot0_yaw) ;

	if (FORMATION_ANG_robot2 >= 2*M_PI)
		FORMATION_ANG_robot2 = FORMATION_ANG_robot2 - 2*M_PI ;
	if (FORMATION_ANG_robot2 <= 2*M_PI)
		FORMATION_ANG_robot2 = FORMATION_ANG_robot2 + 2*M_PI ;

    goal_x_r2 = robot0_x + SWARM_DIST*cos(FORMATION_ANG_robot2) ;
    goal_y_r2 = robot0_y + SWARM_DIST*sin(FORMATION_ANG_robot2) ;

   if (abs(robot0_vx) >= 0.05) 
   {
        omega_n_r2           =  sqrt(pow(robot0_omega,2) + g_r2*pow(robot0_vx,2)) ;
        k1_r2                =  2*0.7*omega_n_r2 ;
        k2_r2                =  2*0.7*omega_n_r2 ;
        k3_r2                =  3*abs(robot0_vx);

        e1_r2                =  cos(robot2_yaw)*(goal_x_r2-robot2_x) + sin(robot2_yaw)*(goal_y_r2-robot2_y) ; 
        e2_r2                = -sin(robot2_yaw)*(goal_x_r2-robot2_x) + cos(robot2_yaw)*(goal_y_r2-robot2_y) ; 
        e3_r2                =  robot0_yaw - robot2_yaw ; 

        // Controller 1 (Linearized controller)
        // vt1_r2               = -k1_r2*e1_r2 ;
        // vt2_r2               = -k3_r2*signum(robot0_vx)*e2_r2 - k2_r2*e3_r2 ;
        // vel_msg_robot2.linear.x  = robot0_vx*cos(e3_r2) - vt1_r2 ;
        // vel_msg_robot2.angular.z = robot0_omega - vt2_r2      ; 

        // Controlle        
        // vt1_r2               = -k1_new*e1_r2 ;
        // vt2_r2               = -k2_new*robot0_vx*e2_r2 - k3_new*robot0_vx*sin(e3_r2) ;
        // vel_msg_robot2.linear.x  = robot0_vx*cos(e3_r2) - vt1_r2 ;
        // vel_msg_robot2.angular.z = robot0_wx - vt2_r2 ; 

        // Controller 3 ( Lyapunov based controller)
        vt1_r2 = -k1_r2*e1_r2                                             ;
        vt2_r2 = -3*robot0_vx*(sin(e3_r2)/e3_r2)*e2_r2 - k2_r2*e3_r2 ;
        vel_msg_robot2.linear.x = robot0_vx*cos(e3_r2) - vt1_r2            ;
        vel_msg_robot2.angular.z = robot0_omega - vt2_r2                      ;     
    }
    else 
    { 
        ROS_INFO_STREAM("Controller 2: robot2 ")   ;   	
        vel_msg_robot2.linear.x = 0 ;
        vel_msg_robot2.angular.z = 0 ;
    }

    cmd_vel_pub_robot2.publish(vel_msg_robot2) ;
}

void formation_control::update()
{
	ros::Time now = ros::Time::now();

	if ( now > t_next) 
	{
		elapsed = now.toSec() - then.toSec()      ; 
		ROS_INFO_STREAM("elapsed =" << elapsed)   ;
		ROS_INFO_STREAM("now =" << now.toSec())   ;
		ROS_INFO_STREAM("then =" << then.toSec()) ;

        geometry_msgs::Twist leader_vel_msg        ;
        leader_vel_msg.linear.x =  0.4             ;
        leader_vel_msg.angular.z = 0               ;
        cmd_vel_leader_pub.publish(leader_vel_msg) ;

        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/elapsed ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/elapsed ;
        robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

		formation_robot1() ;
		//formation_robot2() ;

        robot0_xdot_prev = robot0_xdot      ;
        robot0_ydot_prev = robot0_ydot      ;

        /*
        debug_msg.linear.x = robot0_xdotdot ;
        debug_msg.linear.y = vt2_r2         ;
        debug_msg.linear.z = eta_r2         ;
        debug_msg.angular.x = eta_r2_dot    ;
        debug_msg.angular.y = integral(eta_r2_dot,eta_r2_dot_prev,elapsed)             ;
        debug_msg.angular.z = elapsed       ;
        debug_pub.publish(debug_msg)        ;
        */

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
	   //write_robot1_data("/home/kaustav/catkin_ws/src/multirobot_orca/data/robot1_data.txt") ;
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
  ros::init(argc, argv, "formation_control");
  formation_control formation_one ;
  tf::TransformListener tf;
  formation_one.spin() ; 
  return 0 ;
}



