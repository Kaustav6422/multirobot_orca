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
#include <visualization_msgs/Marker.h>

#include <math.h> 

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
    double time ;
    void visualization() ;
    
private: 
    ros::NodeHandle nh ;
    ros::Publisher cmd_vel_pub_follower1 ;
    ros::Publisher cmd_vel_pub_follower2 ;
    ros::Publisher cmd_vel_leader_pub ;
    ros::Publisher debug_pub ;
    ros::Publisher marker_ref_pub ;
    ros::Publisher marker_robot_pub ;
    ros::Subscriber position_share_sub ;
    ros::Subscriber joy_sub ;
    void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data) ;
    void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy) ;
    void init_variables()  ;
    void update()          ;
    void update_dynamic_noslip() ;
    void update_dynamic_slip() ;
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
    double robot0_wx ;
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

    double robot2_x ;
    double robot2_y ;
    double robot2_xdot ;
    double robot2_ydot ;
    double robot2_xdotdot ;
    double robot2_ydotdot ;
    double robot2_yaw ;
    double robot2_vx ;
    double robot2_wx ;

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
    double e1_dot_robot1, e2_dot_robot1, e3_dot_robot1, e1_dot_robot2, e2_dot_robot2, e3_dot_robot2 ;
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
    //double u1_robot1 ;
    //double u2_robot1 ;

    double eta_r2 ;
    double eta_r2_dot ;
    //double u1_robot2 ;
    //double u2_robot2 ;

    double k1_r1,k2_r1,k3_r1 ;
    double k1_r2,k2_r2,k3_r2 ;

    double rho_r1, gamma_r1, tilda_r1 ;
    double rho_r2, gamma_r2, tilda_r2 ;
    double k1_p, k2_p, k3_p ;

    double k1_5, k2_5, k3_5, k0_5 ; 

    // Chained form
    double X1_robot0_r1 ;
    double X2_robot0_r1 ; 
    double X3_robot0_r1 ;
    double X1_robot0_r2 ;
    double X2_robot0_r2 ; 
    double X3_robot0_r2 ;
    double u1_robot0_r1 ;
    double u2_robot0_r1 ;
    double u1_robot0_r2 ;
    double u2_robot0_r2 ;
    double X1_robot1 ;
    double X2_robot1 ;
    double X3_robot1 ;
    double X1_robot2 ;
    double X2_robot2 ;
    double X3_robot2 ;
    double X1e_robot1 ;
    double X2e_robot1 ;
    double X3e_robot1 ;
    double X1e_robot2 ;
    double X2e_robot2 ;
    double X3e_robot2 ;
    double u1_robot1 ;
    double u2_robot1 ;
    double u1_robot2 ;
    double u2_robot2 ;
    double k1_chained ;
    double k2_chained ;
    double k3_chained ;

    double f ;

    // Dynamic model with no slip
    double mass ;
    double inertia ;
    double F ;
    double Tau ;
    double x, x_p ;
    double y, y_p ;
    double phi, phi_p ;
    double x_e,y_e,phi_e ;    
    double v, v_p ;
    double w, w_p ;
    double vref, wref ;
    double xref, yref, phiref ;
    double xref_dot ,yref_dot ,phiref_dot  ;
    double xref_ddot,yref_ddot,phiref_ddot ;
    double vd, wd ;
    double v_e, w_e, v_e_p, w_e_p, v_e_sum,w_e_sum ;

    // Dynamic model with slip
    double b ; // meters
    double d ; // meters
    double r ; // meters
    double mr ; // kg
    double mw ; // kg
    double Irz ; // kg-m2
    double Iwy ; // kg-m2
    double Iwz ; // kg-m2
    double alpha ; // traction parameter
    double beta ; // traction parameter
};

formation_control::formation_control()
{
    init_variables();

    // Publish
    cmd_vel_leader_pub    = nh.advertise<geometry_msgs::Twist>("/robot_0/mobile_base/commands/velocity",20) ;
    cmd_vel_pub_follower1 = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",20);
    cmd_vel_pub_follower2 = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",20);
    debug_pub             = nh.advertise<geometry_msgs::Twist>("debug",20);  
    marker_ref_pub        = nh.advertise<visualization_msgs::Marker>("trajectory_ref", 20);
    marker_robot_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_robot", 20) ;   

    // Subscribe
    position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
    joy_sub            = nh.subscribe("robot_0/joy",50,&formation_control::joystick_callback,this) ;
}

void formation_control::visualization()
{
    visualization_msgs::Marker reference;
    reference.header.frame_id= "/map";
    reference.header.stamp= ros::Time::now();
    reference.ns= "spheres";
    reference.action= visualization_msgs::Marker::ADD;
    reference.pose.orientation.w= 1.0;
    reference.id = 0;
    reference.type = visualization_msgs::Marker::SPHERE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    reference.scale.x = 0.1;
    reference.scale.y = 0.1;
    reference.scale.z = 0.1;
    // Points are blue
    reference.color.b = 1.0f;
    reference.color.a = 1.0;
    geometry_msgs::Point ref;
    ref.x = xref ; //goal_x_robot1 ;
    ref.y = yref ; //goal_y_robot1 ;
    ref.z = 0;
    reference.points.push_back(ref);
    marker_ref_pub.publish(reference);

    visualization_msgs::Marker robot;
    robot.header.frame_id= "/map";
    robot.header.stamp= ros::Time::now();
    robot.ns= "spheres";
    robot.action= visualization_msgs::Marker::ADD;
    robot.pose.orientation.w= 1.0;
    robot.id = 0;
    robot.type = visualization_msgs::Marker::SPHERE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    robot.scale.x = 0.1;
    robot.scale.y = 0.1;
    robot.scale.z = 0.1;
    // Points are red
    robot.color.r = 1.0f;
    robot.color.a = 1.0;
    geometry_msgs::Point p_robot1;
    p_robot1.x = x ; //goal_x_robot1 ;
    p_robot1.y = y ; //goal_y_robot1 ;
    p_robot1.z = 0;
    robot.points.push_back(p_robot1);
    marker_robot_pub.publish(robot) ;


    f += 0.04;
    
    /*
    visualization_msgs::Marker points, line_strip ;
    points.header.frame_id = line_strip.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    geometry_msgs::Point p_robot1;
    p_robot1.x = goal_x_robot1 ;
    p_robot1.y = goal_y_robot1 ;
    p_robot1.z = 0;
    geometry_msgs::Point p_robot2;
    p_robot2.x = goal_x_robot2 ;
    p_robot2.y = goal_y_robot2 ;
    p_robot2.z = 0;
    points.points.push_back(p_robot1);
    points.points.push_back(p_robot2);
    line_strip.points.push_back(p_robot1);
    line_strip.points.push_back(p_robot2);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    f += 0.04;
    */

}


void formation_control::init_variables()
{
    rate = 20 ;
    time = 0 ;

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
    kd1 = 3 ;
    kd2 = 3 ;

    k1_r1 = 1 ; k2_r1 = 5 ; k3_r1 = 5 ;
    k1_r2 = 1 ; k2_r2 = 5 ; k3_r2 = 5 ;

    eta_r1 = 0.1 ;
    eta_r2 = 0.1 ;

    // Posture stabilization
    rho_r1 = 0 ;
    gamma_r1 = 0 ;
    tilda_r1 = 0 ;
    rho_r2 = 0 ;
    gamma_r2 = 0 ;
    tilda_r2 = 0 ;
    k1_p = 1 ;
    k2_p = 1 ;
    k3_p = 1 ;

    // 
    k0_5 = 1 ;
    k1_5 = 1 ;
    k2_5 = 1 ;

    // Chained form
    k1_chained = 0.5 ;
    k2_chained = 0.5 ;
    k3_chained = 1 ;

    f = 0.0 ;

    mass    = 0.1 ;
    inertia = 0.1 ;
    //F = 0 ;
    //Tau = 0 ;
    x = -1; y = 0; phi = 0 ;
    x_p =0; y_p =0; phi_p =0;
    x_e = 0 ; y_e = 0; phi_e = 0;
    v = 0; w = 0; v_p =0 ; w_p= 0;
    vref = 0 ; wref = 0 ; xref = 0; yref = 0; phiref = 0 ;
    vd = 0 ;wd = 0 ;
    v_e= 0; w_e =0; v_e_p =0; w_e_p =0 ;
    v_e_sum = 0 ; w_e_sum = 0 ;
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
        robot0_wx = data->twist.twist.angular.z ;
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
        robot1_wx = data->twist.twist.angular.z ;
        robot1_xdot = robot1_vx*cos(robot1_yaw) ;
        robot1_ydot = robot1_vx*sin(robot1_yaw) ;
        // Chained form
        X1_robot1  = robot1_yaw ;
        X2_robot1  = robot1_x*cos(robot1_yaw) + robot1_y*sin(robot1_yaw) ;
        X3_robot1  = robot1_x*sin(robot1_yaw) - robot1_y*cos(robot1_yaw) ;

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
        robot2_wx = data->twist.twist.angular.z ;
        robot2_xdot = robot2_vx*cos(robot2_yaw) ;
        robot2_ydot = robot2_vx*sin(robot2_yaw) ;
        // Chained form
        X1_robot2  = robot2_yaw ;
        X2_robot2  = robot2_x*cos(robot2_yaw) + robot2_y*sin(robot2_yaw) ;
        X3_robot2  = robot2_x*sin(robot2_yaw) - robot2_y*cos(robot2_yaw) ;
    }
}

void formation_control::follower1_update()
{
    FORMATION_ANG_robot1 = -(M_PI - SWARM_ANG - robot0_yaw) ;
   
    /*
    if (FORMATION_ANG_robot1 >= 2*M_PI)
        FORMATION_ANG_robot1 = FORMATION_ANG_robot1 - 2*M_PI ;
    if (FORMATION_ANG_robot1 <= 2*M_PI)
        FORMATION_ANG_robot1 = FORMATION_ANG_robot1 + 2*M_PI ; 
    */

    goal_x_robot1 = robot0_x + SWARM_DIST*cos(FORMATION_ANG_robot1) ;
    goal_y_robot1 = robot0_y + SWARM_DIST*sin(FORMATION_ANG_robot1) ;

    omega_n_robot1           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
           
    e1_robot1                =  cos(robot1_yaw)*(goal_x_robot1-robot1_x) + sin(robot1_yaw)*(goal_y_robot1-robot1_y) ; 
    e2_robot1                = -sin(robot1_yaw)*(goal_x_robot1-robot1_x) + cos(robot1_yaw)*(goal_y_robot1-robot1_y) ; 
    e3_robot1                =  robot0_yaw - robot1_yaw  ;

    // Controller 4 (posture stabilization)
    /*
    rho_r1   = sqrt(pow(robot0_x - robot1_x,2) + pow(robot0_y-robot1_y,2)) ;
    gamma_r1 = M_PI + atan2((robot0_y - robot1_y),(robot0_x - robot1_x)) - robot0_yaw ;
    tilda_r1 = gamma_r1 + robot0_yaw ;
    */

    // Chained form (Leader)
    
    X1_robot0_r1 = robot0_yaw ;
    X2_robot0_r1 = goal_x_robot1*cos(robot0_yaw) + goal_y_robot1*sin(robot0_yaw) ;
    X3_robot0_r1 = goal_x_robot1*sin(robot0_yaw) - goal_y_robot1*cos(robot0_yaw) ;
    u1_robot0_r1 = robot0_wx ;
    u2_robot0_r1 = robot0_vx - X3_robot0_r1*u1_robot0_r1 ;
    
    
    if (abs(robot0_vx) > 0.01) 
    {
        ROS_INFO_STREAM("Robot_1: Controller TYPE ONE");

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
        
        u1_robot1            =  robot0_xdotdot + kp1*(goal_x_robot1 - robot1_x) + kd1*(robot0_xdot - robot1_xdot);
        u2_robot1            =  robot0_ydotdot + kp2*(goal_y_robot1 - robot1_y) + kd2*(robot0_ydot - robot1_ydot);
        eta_r1_dot           =  u1_robot1*cos(robot1_yaw)  + u2_robot1*sin(robot1_yaw) ;
        eta_r1               =  eta_r1 + eta_r1_dot*0.05 ; 
        eta_r1               =  max(eta_r1,0.1) ;
        vel_msg_f1.linear.x  =  eta_r1 ; 
        vel_msg_f1.angular.z =  (u2_robot1*cos(robot1_yaw) - u1_robot1*sin(robot1_yaw))/eta_r1 ; 
        

        // Controller 6 (Lyapunov approach using chained form)
        /*
        X1e_robot1 = X1_robot1 - X1_robot0_r1 ;
        X2e_robot1 = X2_robot1 - X2_robot0_r1 ;
        X3e_robot1 = X3_robot1 - X3_robot0_r1 ;
        u1_robot1 = u1_robot0_r1 - k1_chained*(X1e_robot1 + k3_chained*X2_robot1*X3e_robot1) ;
        u2_robot1 = u2_robot0_r1 - k2_chained*X2e_robot1 - k3_chained*u1_robot0_r1*X3e_robot1 ; 
        vel_msg_f1.linear.x  = max(min(u2_robot1 + u1_robot1*X3_robot1,3.0),-3.0) ;    
        vel_msg_f1.angular.z = max(min(u1_robot1,3.0),-3.0) ;    
        */
    }
    else 
    {
        // Controller 4 (Posture stabilization)
        //vel_msg_f1 = k1_p*rho_r1*cos(gamma_r1) ;
        //vel_msg_f1 = k2_p*gamma_r1 + k1_p*sin(gamma_r1)*cos(gamma_r1)*(gamma_r1 + k3_p*tilda_r1)/gamma_r1 ; 

        vel_msg_f1.linear.x  = 0 ; 
        vel_msg_f1.angular.z = 0 ; 
    }
    

    cmd_vel_pub_follower1.publish(vel_msg_f1) ;
}

void formation_control::follower2_update()
{
    FORMATION_ANG_robot2 = -(M_PI + SWARM_ANG - robot0_yaw) ;
    
    /*
    if (FORMATION_ANG_robot2 >= 2*M_PI)
        FORMATION_ANG_robot2 = FORMATION_ANG_robot2 - 2*M_PI ;
    if (FORMATION_ANG_robot2 <= 2*M_PI)
        FORMATION_ANG_robot2 = FORMATION_ANG_robot2 + 2*M_PI ;
    */

    goal_x_robot2 = robot0_x + SWARM_DIST*cos(FORMATION_ANG_robot2) ;
    goal_y_robot2 = robot0_y + SWARM_DIST*sin(FORMATION_ANG_robot2) ;

    omega_n_robot2           =  sqrt(pow(robot0_omega,2) + g*pow(robot0_vx,2)) ;
           
    e1_robot2                =  cos(robot2_yaw)*(goal_x_robot2-robot2_x) + sin(robot2_yaw)*(goal_y_robot2-robot2_y) ; 
    e2_robot2                = -sin(robot2_yaw)*(goal_x_robot2-robot2_x) + cos(robot2_yaw)*(goal_y_robot2-robot2_y) ; 
    e3_robot2                =  robot0_yaw - robot2_yaw  ;

    // Controller 4 (posture stabilization)
    /*
    rho_r2   = sqrt(pow(robot0_x - robot2_x,2) + pow(robot0_y-robot2_y,2)) ;
    gamma_r2 = M_PI + atan2((robot0_y - robot2_y),(robot0_x - robot2_x)) - robot0_yaw ;
    tilda_r2 = atan2((robot0_y - robot2_y),(robot0_x - robot2_x)) ; //gamma_r2 + robot0_yaw ;
    */

    // Chained form (Leader)
    
    X1_robot0_r2 = robot0_yaw ;
    X2_robot0_r2 = goal_x_robot2*cos(robot0_yaw) + goal_y_robot2*sin(robot0_yaw) ;
    X3_robot0_r2 = goal_x_robot2*sin(robot0_yaw) - goal_y_robot2*cos(robot0_yaw) ;
    u1_robot0_r2 = robot0_wx ;
    u2_robot0_r2 = robot0_vx - X3_robot0_r2*u1_robot0_r2 ;
    

    if (abs(robot0_vx) > 0.01) 
    {
        ROS_INFO_STREAM("Controller TYPE ONE");

        // Controller 1 (Linearized controller)
        /*
        vt1_r2               = -k1_r2*e1_robot2 ;
        vt2_r2               = -k3_r2*signum(robot0_vx)*e2_robot2 - k2_r2*e3_robot2 ;
        vel_msg_f2.linear.x  = robot0_vx*cos(e3_robot2) - vt1_r2 ;
        vel_msg_f2.angular.z = robot0_omega - vt2_r2      ; 
        */

        // Controller 2( Lyapunov based controller)
        /*
        vt1_r2                 = -k1_r2*e1_robot2 ;
        vt2_r2                 = -3*robot0_vx*(sin(e3_robot2)/e3_robot2)*e2_robot2 - k2_r2*e3_robot2 ;
        vel_msg_f2.linear.x    = robot0_vx*cos(e3_robot2) - vt1_r2 ;
        vel_msg_f2.angular.z   = robot0_omega - vt2_r2 ;
        */

        // Controller 3 (Dynamic feedback linearization)
        /*
        u1_robot2            =  robot0_xdotdot + kp1*(goal_x_robot2 - robot2_x) + kd1*(robot0_xdot - robot2_xdot);
        u2_robot2            =  robot0_ydotdot + kp2*(goal_y_robot2 - robot2_y) + kd2*(robot0_ydot - robot2_ydot);
        eta_r2_dot           =  u1_robot2*cos(robot2_yaw)  + u2_robot2*sin(robot2_yaw) ;
        eta_r2               =  eta_r2 + eta_r2_dot*0.05 ;
        eta_r2               =  min(eta_r2, 5) ;
        vel_msg_f2.linear.x  =  eta_r2 ;
        vel_msg_f2.angular.z =  (u2_robot2*cos(robot2_yaw) - u1_robot2*sin(robot2_yaw))/eta_r2  ;  
        */

        // Controller 5 (Simultaneous tracking and stabilization)
        /*
        double h = 8*tanh(pow(e1_robot2,2) + pow(e2_robot2,2))*sin(time) ;
        double p = 0 ;
        double alpha = p*h ;
        e1_dot_robot2 =  robot2_wx*e2_robot2 + robot2_vx - robot0_vx*cos(e3_robot2) ; 
        e2_dot_robot2 = -robot2_wx*e1_robot2 + robot0_vx*(sin(e3_robot2)-sin(alpha)) + robot0_vx*sin(alpha) ;
        double alpha_dot = 8*tanh(pow(e1_robot2,2) + pow(e2_robot2,2))*cos(time) + 8*(1/pow(cosh(pow(e1_robot2,2) + pow(e2_robot2,2)),2))*sin(time)*(2*e1_robot2*e1_dot_robot2 + 2*e2_robot2*e2_dot_robot2) ;
        double f1 = (sin(e3_robot2) - sin(alpha))/(e3_robot2 - alpha) ;
        e3_dot_robot2 =  robot2_wx - robot0_wx - alpha_dot ;
        vel_msg_f2.linear.x = (-k1_5*e1_robot2 + robot0_vx*cos(e3_robot2)) ;
        vel_msg_f2.angular.z = -k2_5*(e3_robot2 - alpha) + robot0_wx - k0_5*robot0_vx*e2_robot2*f1 + alpha_dot ;
        */

        // Controller 6 (Lyapunov approach using chained form)
        /*
        X1e_robot2 = X1_robot2 - X1_robot0_r2 ;
        X2e_robot2 = X2_robot2 - X2_robot0_r2 ;
        X3e_robot2 = X3_robot2 - X3_robot0_r2 ;
        u1_robot2 = u1_robot0_r2 - k1_chained*(X1e_robot2 + k3_chained*X2_robot2*X3e_robot2)  ;
        u2_robot2 = u2_robot0_r2 - k2_chained*X2e_robot2 - k3_chained*u1_robot0_r2*X3e_robot2 ;
        vel_msg_f2.linear.x = max(min(u2_robot2 + u1_robot2*X3_robot2,3.0),-3.0)   ;
        vel_msg_f2.angular.z = max(min(u1_robot2,3.0),-3.0) ;
        */
    }
    else 
    {
        // Controller 4 (Posture stabilization)
        //vel_msg_f2.linear.x  = k1_p*rho_r2*cos(gamma_r2) ;
        //vel_msg_f2.angular.z = k2_p*gamma_r2 + k1_p*sin(gamma_r2)*cos(gamma_r2)*(gamma_r2 + k3_p*tilda_r2)/gamma_r2 ;

        vel_msg_f2.linear.x  = 0 ; 
        vel_msg_f2.angular.z =  0 ; 
    }
    
    
    cmd_vel_pub_follower2.publish(vel_msg_f2) ;
}

void formation_control::update()
{
    ros::Time now = ros::Time::now();

    if ( now > t_next) 
    {
        elapsed = now.toSec() - then.toSec(); 
        time = time + 0.05 ;
        //ROS_INFO_STREAM("elapsed =" << elapsed);
        //ROS_INFO_STREAM("now =" << now.toSec());
        //ROS_INFO_STREAM("then =" << then.toSec());

        
        
        // Lissajous curve
        double a = 0.75 ;
        double b = 1 ;
        double xrdot = -a*sqrt(b)*cos(sqrt(b)*time) ;
        double yrdot = 0.5*a*sqrt(b)*cos(0.25*sqrt(b)*time) ;
        double xrddot = a*b*sin(sqrt(b)*time) ;
        double yrddot = -0.25*a*b*sin(0.25*sqrt(b)*time) ;
        double vr = sqrt(pow(xrdot,2) + pow(yrdot,2)) ;
        double wr = (xrddot*yrdot - yrddot*xrdot)/vr ;
        
        /*
        geometry_msgs::Twist leader_vel_msg ;
        leader_vel_msg.linear.x   = 0.7 ;  // exp(-0.1*time)        ;
        leader_vel_msg.angular.z  = -0.1 ; //-exp(-time)            ;
        cmd_vel_leader_pub.publish(leader_vel_msg) ;
        */

        robot0_xdotdot = (robot0_xdot - robot0_xdot_prev)/elapsed ;
        robot0_ydotdot = (robot0_ydot - robot0_ydot_prev)/elapsed ;
        robot0_omega   = (robot0_xdot*robot0_ydotdot - robot0_xdotdot*robot0_ydot)/pow(robot0_vx,2) ;

        follower1_update() ;
        follower2_update() ;

        robot0_xdot_prev = robot0_xdot ;
        robot0_ydot_prev = robot0_ydot ;

        // u1_robot = u1_robot0_r2 - k1_chained*(X1e_robot2 + k3_chained*X2_robot2*X3e_robot2)
        
        debug_msg.linear.x =  0        ;
        debug_msg.linear.y =  0        ;
        debug_msg.linear.z =  0            ;
        debug_msg.angular.x = X3_robot0_r2               ;
        debug_msg.angular.y = X3_robot2                   ;
        debug_msg.angular.z = X3e_robot2                  ;
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

void formation_control::update_dynamic_noslip()
{
    ros::Time now = ros::Time::now();

    //if ( now > t_next) 
    //{

        double k1,k2,k3 ;
        k1 = 1;k2 = 6.5;k3 = 6.5;
        double kp,kd,ki ;
        kp = 1;kd = 1;ki = 10 ;

        double dt ;
        dt = 0.05 ; //now.toSec() - then.toSec(); 
        time = time + 0.05 ;
        ROS_INFO_STREAM("elapsed =" << dt);
        //ROS_INFO_STREAM("now =" << now.toSec());
        //ROS_INFO_STREAM("then =" << then.toSec());

        // Lissajous curve
        
        double A,B ;
        A = 1.5 ;
        B = 0.5 ;
        xref = A*sin(B*time) ;
        yref = 2*A*sin(0.25*B*time) ;
        xref_dot = A*B*cos(B*time) ;
        yref_dot = 0.5*A*B*cos(0.25*B*time) ;
        phiref = atan2(yref_dot,xref_dot) ;
        xref_ddot = -A*B*B*sin(B*time) ;
        yref_ddot = -0.5*0.25*A*B*B*sin(0.25*B*time) ;
        vref = sqrt(pow(xref_dot,2) + pow(yref_dot,2)) ;
        wref = (xref_ddot*yref_dot - yref_ddot*xref_dot)/pow(vref,2) ;
        

        // Straight line
        /*
        vref = 0.5 ;
        wref = 0.25 ;
        //phiref = 0 ;
        xref = xref + vref*cos(phiref)*dt ;
        yref = yref + vref*sin(phiref)*dt ;
        phiref = phiref + wref*dt ;
        */


        x_e   =  cos(phi)*(xref - x) + sin(phi)*(yref - y) ;
        y_e   = -sin(phi)*(xref - x) + cos(phi)*(yref - y) ;
        phi_e =  phiref - phi ; 

        vd    = vref*cos(phi_e) + k1*x_e ;
        wd    = wref + k3*signum(vref)*y_e + k2*phi_e ;

        v_e   = vd - v ;
        v_e_sum += v_e*dt ;
        w_e   = wd - w ; 
        w_e_sum += w_e*dt ;

        F   = kp*v_e + ki*v_e_sum  ;
        Tau = kp*w_e + ki*w_e_sum  ;

        debug_msg.linear.x =  F        ;
        debug_msg.linear.y =  Tau      ;
        debug_msg.linear.z =  0        ;
        debug_msg.angular.x = x_e               ;
        debug_msg.angular.y = y_e                ;
        debug_msg.angular.z = phi_e               ;
        debug_pub.publish(debug_msg)                   ;
        
        v   =  v + (F/mass)*dt ;//v_p + (F/mass)*dt       ;
        w   =  w + (Tau/inertia)*dt ; //w_p + (Tau/inertia)*dt  ;
        x   =  x + v*cos(phi)*dt ; //x_p   + v*cos(phi)*dt ;
        y   =  y + v*sin(phi)*dt ; //y_p   + v*sin(phi)*dt ;
        phi =  phi + w*dt ; //phi_p + w*dt          ;
        
        then = now;
        ros::spinOnce();
    /*    
    }
     else 
    {
        ROS_INFO_STREAM("LOOP MISSED");
    } 
    */
}

void formation_control::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
       //update();
       update_dynamic_noslip() ;
       visualization() ;
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
