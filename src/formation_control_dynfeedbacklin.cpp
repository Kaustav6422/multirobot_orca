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
#include <sensor_msgs/Joy.h>

using namespace std ;

/*
#include "IPlanarGridMap.h"
#include "PlanarGridBinaryMap.h"
#include "PlanarGridContainer.h"
#include "PlanarGridIndex.h"
#include "PlanarGridOccupancyMap.h"private: 
*/

#define MAX_LINEAR_VEL 10.0
#define MAX_ANGULAR_VEL 10.0 

class formation_control
{
public:
    formation_control()    ;
    void spin()            ;
    int signum(double val) ;
private: 
    ros::NodeHandle nh ;
    ros::Publisher cmd_vel_pub_follower1 ;
    ros::Publisher cmd_vel_pub_follower2 ;
    ros::Publisher cmd_vel_leader_pub ;
    ros::Publisher debug_pub ;
    ros::Subscriber position_share_sub ;
    ros::Subscriber joy_sub ;
    void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data) ;
    void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy) ;
    void init_variables()  ;
    void update()          ;
    void follower1_update() ;
    void follower2_update() ; 

    geometry_msgs::Twist debug_msg      ;
    geometry_msgs::Twist vel_msg_f1     ;
    geometry_msgs::Twist vel_msg_f2     ; 

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
    double robot0_omega ;

    double robot1_x ;
    double robot1_y ;
    double robot1_xdot ;
    double robot1_ydot ;
    double robot1_xdotdot ;
    double robot1_ydotdot ;
    double robot1_yaw ;
    double robot1_vx ;

    double robot2_x ;
    double robot2_y ;
    double robot2_xdot ;
    double robot2_ydot ;
    double robot2_xdotdot ;
    double robot2_ydotdot ;
    double robot2_yaw ;
    double robot2_vx ;

    double error_dist_prev ;
    double error_dist ;
    double error_dist_rate ;
    double error_yaw ;
    double error_yaw_rate ;

    double goal_x_robot1 ;
    double goal_y_robot1 ;
    double goal_x_robot2 ;
    double goal_y_robot2 ;

    double g        ;
    double omega_n_robot1,omega_n_robot2  ;
    double k1,k2,k3 ;
    double e1_robot1,e2_robot1,e3_robot1,e1_robot2,e2_robot2,e3_robot2 ;
    double vt1_r1, vt2_r1, vt1_r2, vt2_r2 ;

    double k_s ;
    double k_theta ;
    double k_orientation ; 
    double e_s_robot1 ;
    double e_theta_robot1 ;
    double e_orientation_robot1 ;
    double e_s_robot2 ;
    double e_theta_robot2 ;
    double e_orientation_robot2 ;
    double theta_robot1 ;
    double theta_robot2 ;
    double delta_l_robot1 ;
    double delta_l_robot2 ;

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
    ros::Time then;

    double kp1,kp2,kd1,kd2 ;

    double eta_r1 ;
    double eta_r1_dot ;
    double u1_robot1 ;
    double u2_robot1 ;

    double eta_r2 ;
    double eta_r2_dot ;
    double u1_robot2 ;
    double u2_robot2 ;

    double k1_r1,k2_r1,k3_r1 ;
    double k1_r2,k2_r2,k3_r2 ;
};

formation_control::formation_control()
{
    init_variables();

    // Publish
    cmd_vel_leader_pub    = nh.advertise<geometry_msgs::Twist>("/robot_0/mobile_base/commands/velocity",20) ;
    cmd_vel_pub_follower1 = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",20);
    cmd_vel_pub_follower2 = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",20);
    debug_pub             = nh.advertise<geometry_msgs::Twist>("debug",20);  

    // Subscribe
    position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
    joy_sub            = nh.subscribe("robot_0/joy",50,&formation_control::joystick_callback,this) ;
}

void formation_control::init_variables()
{
    rate = 20 ;

    SWARM_DIST = 1 ;
    SWARM_ANG = 0.785 + 1.57 ;
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
    robot0_vx  = 0;   //correction
    robot0_omega = 0 ; // angular velocity curvature

    robot1_x = 0 ;
    robot1_y = 0 ;
    robot1_xdot = 0 ;
    robot1_ydot = 0 ;
    robot1_xdotdot = 0 ;
    robot1_ydotdot = 0 ;
    robot1_yaw = 0 ;
    robot1_vx = 0 ;

    robot2_x = 0 ;
    robot2_y = 0 ;
    robot2_xdot = 0 ;
    robot2_ydot = 0 ;
    robot2_xdotdot = 0 ;
    robot2_ydotdot = 0 ;
    robot2_yaw = 0 ;
    robot2_vx = 0 ;

    error_dist_prev = 0;
    error_dist = 0 ;
    error_dist_rate = 0 ;
    error_yaw = 0 ;
    error_yaw_rate = 0 ;

    goal_x_robot1 = 0 ;
    goal_y_robot1 = 0 ;
    goal_x_robot2 = 0 ;
    goal_y_robot2 = 0 ;

    g = 5 ;
    omega_n_robot1 = 0 ;
    omega_n_robot2 = 0 ;
    k1 = 1 ;
    k2 = 10 ;
    k3 = 1 ;
    e1_robot1 = 0 ;
    e2_robot1 = 0 ;
    e3_robot1 = 0 ;
    e1_robot2 = 0 ;
    e2_robot2 = 0 ;
    e3_robot2 = 0 ;
    k_s = 2 ;
    k_theta = 4 ;
    k_orientation = 1 ;
    e_s_robot1 = 0 ;
    e_theta_robot1 = 0 ;
    e_orientation_robot1 = 0;
    e_s_robot2 = 0 ;
    e_theta_robot2 = 0 ;
    e_orientation_robot2 = 0;
    theta_robot1 = 0 ;
    theta_robot2 = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();

    kp1 = 1 ;
    kp2 = 1 ;
    kd1 = 1 ;
    kd2 = 1 ;

    k1_r1 = 1 ; k2_r1 = 1 ; k3_r1 = 1 ;
    k1_r2 = 1 ; k2_r2 = 1 ; k3_r2 = 1 ;
}

int formation_control::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
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
        robot1_xdot = robot1_vx*cos(robot1_yaw) ;
        robot1_ydot = robot1_vx*sin(robot1_yaw) ;
    }

    if (data->robot_id == "robot_2")
    {
        robot2_x = data->pose.pose.position.x ;
        robot2_y = data->pose.pose.position.y ;
        tf::Quaternion q_robot2(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
        tf::Matrix3x3 m_robot2(q_robot2);
        double roll, pitch, yaw;
        m_robot2.getRPY(roll, pitch, yaw);
        robot2_yaw = yaw ;
        robot2_vx = data->twist.twist.linear.x ;
        robot2_xdot = robot2_vx*cos(robot2_yaw) ;
        robot2_ydot = robot2_vx*sin(robot2_yaw) ;
    }
}

void formation_control::follower1_update()
{
    FORMATION_ANG_robot1 = -(M_PI - SWARM_ANG - robot0_yaw) ;

    if (FORMATION_ANG_robot1 >= 2*M_PI)
        FORMATION_ANG_robot1 = FORMATION_ANG_robot1 - 2*M_PI ;
    if (FORMATION_ANG_robot1 <= 2*M_PI)
        FORMATION_ANG_robot1 = FORMATION_ANG_robot1 + 2*M_PI ; 

    goal_x_robot1 = robot0_x + SWARM_DIST*cos(FORMATION_ANG_robot1) ;
    goal_y_robot1 = robot0_y + SWARM_DIST*sin(FORMATION_ANG_robot1) ;
        
    if (abs(robot0_vx) > 0.05) 
    {
        ROS_INFO_STREAM("Robot_1: Controller TYPE ONE");

        omega_n_robot1           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
           
        e1_robot1                =  cos(robot1_yaw)*(goal_x_robot1-robot1_x) + sin(robot1_yaw)*(goal_y_robot1-robot1_y) ; 
        e2_robot1                = -sin(robot1_yaw)*(goal_x_robot1-robot1_x) + cos(robot1_yaw)*(goal_y_robot1-robot1_y) ; 
        e3_robot1                =  robot0_yaw - robot1_yaw  ;


        // Controller 2 ( Linearized Controller)
        /*
        vt1_r1               = -k1_r1*e1_robot1 ;
        vt2_r1               = -k3_r1*signum(robot0_vx)*e2_robot1 - k2_r1*e3_robot1 ;
        vel_msg_f1.linear.x  = robot0_vx*cos(e3_robot1) - vt1_r1 ;
        vel_msg_f1.angular.z = robot0_omega - vt2_r1      ; 
        */

        // Controller 2( Lyapunov based controller)
        /*
        vt1_r1                   = -k1_r1*e1_robot1 ;
        vt2_r1                   = -3*robot0_vx*(sin(e3_robot1)/e3_robot1)*e2_robot1 - k2_r1*e3_robot1 ;
        vel_msg_f1.linear.x  = robot0_vx*cos(e3_robot1) - vt1_r1 ;
        vel_msg_f1.angular.z = robot0_omega - vt2_r1 ;
        */

        // Controller 3 ( Dynamic feedback linearization)
        /*
        u1_robot1         =  robot0_xdotdot + kp1*(goal_x_robot1 - robot1_x) + kd1*(robot0_xdot - robot1_xdot);
        u2_robot1         =  robot0_ydotdot + kp2*(goal_y_robot1 - robot1_y) + kd2*(robot0_ydot - robot1_ydot);
        eta_r1_dot               =  u1_robot1*cos(robot1_yaw)  + u2_robot1*sin(robot1_yaw) ;
        eta_r1                   =  eta_r1 + eta_r1_dot*0.05 ; //integral(eta_r1_dot,eta_r1_dot_prev, elapsed) ;
        vel_msg_f1.linear.x  =  eta_r1 ; 
        vel_msg_f1.angular.z =  0 ; //(vt2_r1*cos(robot1_yaw) - vt1_r1*sin(robot1_yaw))/eta_r1 ; 
        */                                            ;                                              
    }
    else 
    {   
        vel_msg_f1.linear.x  = 0 ; 
        vel_msg_f1.angular.z = 0 ; 
    }
    cmd_vel_pub_follower1.publish(vel_msg_f1) ;
}

void formation_control::follower2_update()
{
    FORMATION_ANG_robot2 = -(M_PI + SWARM_ANG - robot0_yaw) ;

    if (FORMATION_ANG_robot2 >= 2*M_PI)
        FORMATION_ANG_robot2 = FORMATION_ANG_robot2 - 2*M_PI ;
    if (FORMATION_ANG_robot2 <= 2*M_PI)
        FORMATION_ANG_robot2 = FORMATION_ANG_robot2 + 2*M_PI ;

    goal_x_robot2 = robot0_x + SWARM_DIST*cos(FORMATION_ANG_robot2) ;
    goal_y_robot2 = robot0_y + SWARM_DIST*sin(FORMATION_ANG_robot2) ;
      
    if (abs(robot0_vx) > 0.01) 
    {
        ROS_INFO_STREAM("Controller TYPE ONE");

        omega_n_robot2           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
           
        e1_robot2                =  cos(robot2_yaw)*(goal_x_robot2-robot2_x) + sin(robot2_yaw)*(goal_y_robot2-robot2_y) ; 
        e2_robot2                = -sin(robot2_yaw)*(goal_x_robot2-robot2_x) + cos(robot2_yaw)*(goal_y_robot2-robot2_y) ; 
        e3_robot2                =  robot0_yaw - robot2_yaw  ; 

        // Controller 1 (Linearized controller)
        /*
        vt1_r2               = -k1_r2*e1_robot2 ;
        vt2_r2               = -k3_r2*signum(robot0_vx)*e2_robot2 - k2_r2*e3_robot2 ;
        vel_msg_f2.linear.x  = robot0_vx*cos(e3_robot2) - vt1_r2 ;
        vel_msg_f2.angular.z = robot0_omega - vt2_r2      ; 
        */

        // Controller 2( Lyapunov based controller)
        /*
        vt1_r2                   = -k1_r2*e1_robot2 ;
        vt2_r2                   = -3*robot0_vx*(sin(e3_robot2)/e3_robot2)*e2_robot2 - k2_r2*e3_robot2 ;
        vel_msg_f2.linear.x  = robot0_vx*cos(e3_robot2) - vt1_r2 ;
        vel_msg_f2.angular.z = robot0_omega - vt2_r2 ;
        */

        // Controller 3 (Dynamic feedback linearization)
        /*
        u1_robot2         =  robot0_xdotdot + kp1*(goal_x_robot2 - robot2_x) + kd1*(robot0_xdot - robot2_xdot);
        u2_robot2         =  robot0_ydotdot + kp2*(goal_y_robot2 - robot2_y) + kd2*(robot0_ydot - robot2_ydot);
        eta_r2_dot        =  u1_robot2*cos(robot2_yaw)  + u2_robot2*sin(robot2_yaw) ;
        eta_r2            =  eta_r2 + eta_r2_dot*0.05 ; 
        vel_msg_f2.linear.x = eta_r2 ;
        vel_msg_f2.angular.z = 0 ;                                              
        */
    }
    else 
    {
        vel_msg_f2.linear.x  = 0 ; 
        vel_msg_f2.angular.x =  0 ; 
    }
    cmd_vel_pub_follower2.publish(vel_msg_f2) ;
}


void formation_control::update()
{
    ros::Time now = ros::Time::now();

    if ( now > t_next) 
    {
        elapsed = now.toSec() - then.toSec(); 
        ROS_INFO_STREAM("elapsed =" << elapsed);
        ROS_INFO_STREAM("now =" << now.toSec());
        ROS_INFO_STREAM("then =" << then.toSec());

        geometry_msgs::Twist leader_vel_msg        ;
        leader_vel_msg.linear.x =  0.4             ;
        leader_vel_msg.angular.z = 0               ;
        cmd_vel_leader_pub.publish(leader_vel_msg) ;

        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/elapsed ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/elapsed ;
        robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

        follower1_update() ;
        follower2_update() ;

        robot0_xdot_prev = robot0_xdot ;
        robot0_ydot_prev = robot0_ydot ;
        
        debug_msg.linear.x =  eta_r1          ;
        debug_msg.linear.y =  eta_r1_dot         ;
        debug_msg.linear.z =  elapsed              ;
        debug_msg.angular.x = 0               ;
        debug_msg.angular.y = u1_robot1                   ;
        debug_msg.angular.z = u2_robot1                   ;
        debug_pub.publish(debug_msg)                   ;
       
        /*
        FORMATION_ANG = SWARM_ANG + robot0_yaw ;

        if (FORMATION_ANG >= 2*M_PI)
            FORMATION_ANG = FORMATION_ANG - 2*M_PI ;
        if (FORMATION_ANG <= 2*M_PI)
            FORMATION_ANG = FORMATION_ANG + 2*M_PI ; 

        goal_x = robot0_x + SWARM_DIST*cos(FORMATION_ANG) ;
        goal_y = robot0_y + SWARM_DIST*sin(FORMATION_ANG) ;
        
        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/elapsed ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/elapsed ;

        if (abs(robot0_vx) > 0.01) 
        {
            ROS_INFO_STREAM("Controller TYPE ONE");

            robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

            omega_n           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
           
            e1                =  cos(robot1_yaw)*(goal_x-robot1_x) + sin(robot1_yaw)*(goal_y-robot1_y) ; 
            e2                = -sin(robot1_yaw)*(goal_x-robot1_x) + cos(robot1_yaw)*(goal_y-robot1_y) ; 
            e3                =  robot0_yaw - robot1_yaw                                               ;                                              

            robot0_xdot_prev = robot0_xdot      ;
            robot0_ydot_prev = robot0_ydot      ;
            
            vel_msg.linear.x  = min(robot0_vx*cos(e3) + k1*e1, MAX_LINEAR_VEL)        ; 
            vel_msg.angular.z = min(robot0_omega + k2*robot0_vx*e2 + k3*robot0_vx*sin(e3), MAX_ANGULAR_VEL) ; 
            
            debug_msg.linear.x =  robot0_yaw               ;
            debug_msg.linear.y =  robot1_yaw               ;
            debug_msg.linear.z =  robot0_omega             ;
            debug_msg.angular.x = robot1_y                 ;
            debug_msg.angular.y = goal_x                   ;
            debug_msg.angular.z = goal_y                   ;
            debug_pub.publish(debug_msg)                   ;
        }
        else 
        {
            ROS_INFO_STREAM("Controller TYPE TWO");
            delta_l = sqrt(pow(goal_y-robot1_y,2) + pow(goal_x-robot1_x,2))   ;
            theta = atan2(goal_y-robot1_y,goal_x-robot1_x)                    ;
            e_theta = theta - robot1_yaw ;
            e_orientation = robot0_yaw - robot1_yaw ;
            e_s = delta_l*cos(e_theta) ;
            vel_msg.linear.x  = k_s*e_s ;
            if (delta_l > 0.01)
            {
                vel_msg.angular.z = k_theta*e_theta ;
            }
            else
            {
                vel_msg.angular.z = k_orientation*e_orientation ;
            }
        }
        cmd_vel_pub.publish(vel_msg) ;
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
