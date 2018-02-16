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
#include "Eigen/Dense"
#include <math.h> 

using namespace std ;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
    ros::Publisher debug_pub ;
    ros::Publisher marker_ref_pub ;
    ros::Publisher marker_robot_pub ;
    void init_variables()  ;
     
    geometry_msgs::Twist debug_msg      ;

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
    ros::Time then;

    double f ;

    /*
    void update_dynamic_noslip() ;
    void update_dynamic_slip() ;
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
    */

    void reference_trajectory(double time, double dt) ;
    void outerloop_controller(double dt)   ;
    void innerloop_controller(double dt)   ;
    void dynamics_noslip(double dt)        ;
    void kinematics_noslip(double dt)      ;
    void update_dynamic_complex_noslip()   ;
    
    void debug() ;
    // Dynamic properties
    double mb ;
    double mw ;
    double mt ;
    double rho ;
    double W ;
    double e ;
    double d ;
    double h ;
    double Ib ;
    double Iwz ; 
    double Iwy ;
    double It ;
    double mu_s ;
    double mu_k ;
    MatrixXd M ;
    MatrixXd C ;
    MatrixXd B ;
    MatrixXd A ;
    MatrixXd S ;
    MatrixXd Sdot ;
    MatrixXd Mhat ;
    MatrixXd Vhat ;
    MatrixXd Bhat ;
    VectorXd tau ;
    VectorXd omega ;
    VectorXd omegadot ;
    // Kinematic properties
    double x ;
    double y ;
    double theta, theta_prev, thetadot ;
    double phi1, phi2 ;
    VectorXd q ;
    VectorXd qdot ;
    // Reference trajectory parameters
    double xref ;
    double yref ;
    double xref_dot ;
    double yref_dot ;
    double thetaref ;
    double xref_ddot ;
    double yref_ddot ;
    double vref ;
    double wref ; 
    // Outerloop controller
    double vd,wd,vd_dot,wd_dot,vd_prev,wd_prev ;
    double v,w ;
    double x_e ;
    double y_e ;
    double theta_e ;
    
};

int formation_control::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

formation_control::formation_control()
{
    init_variables();

    // Publish
    debug_pub             = nh.advertise<geometry_msgs::Twist>("debug",20);  
    marker_ref_pub        = nh.advertise<visualization_msgs::Marker>("trajectory_ref", 20);
    marker_robot_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_robot", 20) ;    
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
    ref.x = xref ; 
    ref.y = yref ; 
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
    p_robot1.x = x ; 
    p_robot1.y = y ; 
    p_robot1.z = 0;
    robot.points.push_back(p_robot1);
    marker_robot_pub.publish(robot) ;


    f += 0.04;
}

void formation_control::init_variables()
{
    rate = 20 ;
    time = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();

    f = 0.0 ;
    
    /*
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
    */

    // Reference trajectory
    vref = 0 ; wref = 0 ; xref = 0; yref = 0; thetaref = 0 ;
    // Dynamic properties (From Torres's thesis pg 45)
    mb = 1.5 ;     // kgs
    mw = 0.064 ;   // kgs
    mt = mb + 2*mw ;
    rho = 0.0365 ; // meters
    W = 0.105 ;    // meters
    e = 0.095 ;    // meters
    d = 0.055 ;    // meters
    h = 0.0216 ;   // meters
    Ib  = 0.009753 ; // kg-m2
    Iwz = 0.000584 ; // kg-m2
    Iwy = 0.001168 ; // kg-m2
    It = Ib + 2*pow(Iwz,2) + 2*mw*pow(w,2) + 2*mw*pow(d,2) ;
    mu_s = 0.241 ;  // coefficient of static friction 
    mu_k = 0.241 ;  // coefficient of kinetic friction
    M = MatrixXd(5, 5);
    M << 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0;
    C = MatrixXd(5,5) ;
    C << 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0;
    B = MatrixXd(5,2) ;
    B << 0.0, 0.0, 
         0.0, 0.0, 
         0.0, 0.0, 
         1.0, 0.0, 
         0.0, 1.0;
    A = MatrixXd(5,3) ;
    A << 0, 0, 0, 
         0, 0, 0, 
         0, 0, 0, 
         0, 0, 0, 
         0, 0, 0; 
    S = MatrixXd(5,2) ;
    S << 0, 0, 
         0, 0, 
         0, 0, 
         0, 0, 
         0, 0; 
    Sdot = MatrixXd(5,2) ;
    Sdot << 0, 0, 
            0, 0, 
            0, 0, 
            0, 0, 
            0, 0;
    omega = VectorXd(2) ;
    omega << 0.0, 0.0 ;
    omegadot = VectorXd(2) ;
    omegadot << 0.0, 0.0 ;
    tau = VectorXd(2) ;
    tau << 0.0, 0.0 ; 
    Mhat = S.transpose()*M*S ;
    Vhat = S.transpose()*M*Sdot + S.transpose()*C*S ;
    Bhat = S.transpose()*B ;  
    // Kinematic properties
    x = 0.0 ;
    y = 0.0 ;
    theta = 0.0 ;
    theta_prev = 0.0 ;
    thetadot = 0.0 ;
    phi1  = 0.0 ;
    phi2  = 0.0 ;
    q = VectorXd(5) ;
    q << x, y, theta, phi1, phi2 ;
    qdot = VectorXd(5) ;
    qdot << 0.0, 0.0, 0.0, 0.0, 0.0 ;
    
}

void formation_control::reference_trajectory(double time, double dt)
{
    // Lissajous curve
    double Aa,Bb ;
    Aa = 1.5 ;
    Bb = 0.5 ;
    xref = Aa*sin(Bb*time) ;
    yref = 2*Aa*sin(0.25*Bb*time) ;
    xref_dot = Aa*Bb*cos(Bb*time) ;
    yref_dot = 0.5*Aa*Bb*cos(0.25*Bb*time) ;
    thetaref = atan2(yref_dot,xref_dot) ;
    xref_ddot = -Aa*Bb*Bb*sin(Bb*time) ;
    yref_ddot = -0.5*0.25*Aa*Bb*Bb*sin(0.25*Bb*time) ;
    vref = sqrt(pow(xref_dot,2) + pow(yref_dot,2)) ;
    wref = (xref_ddot*yref_dot - yref_ddot*xref_dot)/pow(vref,2) ;
    

    // Curved path
    /*
    vref = 0.25   ;
    wref = 0.1    ;
    //thetaref = 0  ;
    xref = xref + vref*cos(thetaref)*dt ;
    yref = yref + vref*sin(thetaref)*dt ;
    thetaref = thetaref + wref*dt ;
    */

    // Straight path
    /*
    vref = 0.5  ;
    wref = 0    ;
    thetaref = 0 ;
    xref = xref + vref*cos(thetaref)*dt ;
    yref = yref + vref*sin(thetaref)*dt ;
    */
}

void formation_control::outerloop_controller(double dt)
{
    double k1,k2,k3 ;
    k1 = 5.5;   // 6.5 for straight line 
    k2 = 5.5; // 6.5 for straight line 
    k3 = 9.5; // 6.5 for straight line
    double x_e ;
    double y_e ;
    double theta_e ;    
    x_e     =  cos(theta)*(xref - x) + sin(theta)*(yref - y) ;
    y_e     = -sin(theta)*(xref - x) + cos(theta)*(yref - y) ;
    theta_e =  thetaref - theta ; 
    vd    = vref*cos(theta_e) + k1*x_e ;
    wd    = wref + k3*signum(vref)*y_e + k2*theta_e ;
}

void formation_control::innerloop_controller(double dt)
{
    double kp_v,ki_v ;
    double kp_w,ki_w ;
    kp_v = 2.5 ; // 2.5 for straight line
    ki_v = 2.5 ; // 2.5 for straight line
    kp_w = 1 ;   // 1 for straight line
    ki_w = 1 ;   // 1 for straight line
    double v_e, v_e_sum, w_e, w_e_sum ;
    v_e   = vd - v ;
    w_e   = wd - w ; 
    v_e_sum += v_e*dt ;
    w_e_sum += w_e*dt ;
    double F, Tau, tau1, tau2 ;
    F   = kp_v*v_e ; //+ ki_v*v_e_sum  ;
    Tau = kp_w*w_e ; //+ ki_w*w_e_sum  ;
    tau1 = (rho/2)*(F + Tau/W) ;
    tau2 = (rho/2)*(F - Tau/W) ;
    tau << tau1, tau2 ;
}

void formation_control::dynamics_noslip(double dt)
{
   // Dynamic model
    M <<                 mt,                  0.0,  2*mw*d*sin(theta),   0.0,   0.0,
                          0.0,                 mt, -2*mw*d*cos(theta),   0.0,   0.0,
          2*mw*d*sin(theta), -2*mw*d*cos(theta),                 It,   0.0,   0.0,
                          0.0,                  0.0,                  0.0, Iwy,   0.0,
                          0.0,                  0.0,                  0.0,   0.0, Iwy;                      
    C <<  0.0, 0.0, 2*mw*d*cos(theta)*thetadot, 0.0, 0.0,
          0.0, 0.0, 2*mw*d*sin(theta)*thetadot, 0.0, 0.0,
          0.0, 0.0,                        0.0, 0.0, 0.0,
          0.0, 0.0,                        0.0, 0.0, 0.0,
          0.0, 0.0,                        0.0, 0.0, 0.0;    
    B << 0.0, 0.0,
         0.0, 0.0,
         0.0, 0.0,
         1.0, 0.0,
         0.0, 1.0; 
    A <<  cos(theta), sin(theta),  W, -rho,    0.0,
          cos(theta), sin(theta), -W,    0.0, -rho,
         -sin(theta), cos(theta), -d,    0.0,    0.0;
    S << 0.5*rho*cos(theta), 0.5*rho*cos(theta), 
         0.5*rho*sin(theta), 0.5*rho*sin(theta),
                  0.5*rho/W,         -0.5*rho/W,
                        1.0,                0.0,
                        0.0,                1.0;                      
    Sdot << -0.5*rho*sin(theta)*thetadot, -0.5*rho*sin(theta)*thetadot, 
             0.5*rho*cos(theta)*thetadot,  0.5*rho*cos(theta)*thetadot,
                                   0.0,                        0.0,
                                   0.0,                        0.0, 
                                   0.0,                        0.0;                                                   
    Mhat = S.transpose()*M*S ;
    Vhat = S.transpose()*M*Sdot + S.transpose()*C*S ;
    Bhat = S.transpose()*B ;
    omegadot = Mhat.inverse()*(Bhat*tau - Vhat*omega) ;
    omega = omega + omegadot*dt ;
    v = 0.5*omega(0)*rho + 0.5*omega(1)*rho ;
    w = rho*(omega(0) - omega(1))/(2*W) ;
}

void formation_control::kinematics_noslip(double dt)
{
    qdot = S*omega ;
    q = q + qdot*dt ;
    x = q(0) ;
    y = q(1) ;
    theta = q(2) ;
    thetadot = (theta - theta_prev)/dt ;
    phi1  = q(3) ;
    phi2  = q(4) ;
    theta_prev = theta ;
}

void formation_control::debug()
{
    debug_msg.linear.x =  x       ;
    debug_msg.linear.y =  y        ;
    debug_msg.linear.z =  theta    ;
    debug_msg.angular.x = 0    ;
    debug_msg.angular.y = vd        ;
    debug_msg.angular.z = wd       ;
    debug_pub.publish(debug_msg)    ;
}

void formation_control::update_dynamic_complex_noslip()
{
    ros::Time now = ros::Time::now();

    double dt ;
    dt = 0.05 ; //now.toSec() - then.toSec(); 
    time = time + 0.05 ;
    //ROS_INFO_STREAM("elapsed =" << dt);

    // Reference trajectory
    /*
    double Aa,Bb ;
    Aa = 1.5 ;
    Bb = 0.5 ;
    xref = Aa*sin(Bb*time) ;
    yref = 2*Aa*sin(0.25*Bb*time) ;
    xref_dot = Aa*Bb*cos(Bb*time) ;
    yref_dot = 0.5*Aa*Bb*cos(0.25*Bb*time) ;
    thetaref = atan2(yref_dot,xref_dot) ;
    xref_ddot = -Aa*Bb*Bb*sin(Bb*time) ;
    yref_ddot = -0.5*0.25*Aa*Bb*Bb*sin(0.25*Bb*time) ;
    vref = sqrt(pow(xref_dot,2) + pow(yref_dot,2)) ;
    wref = (xref_ddot*yref_dot - yref_ddot*xref_dot)/pow(vref,2) ;
    
    
    vref = 0.25   ;
    wref = 0.1    ;
    //thetaref = 0  ;
    xref = xref + vref*cos(thetaref)*dt ;
    yref = yref + vref*sin(thetaref)*dt ;
    thetaref = thetaref + wref*dt ;
    

    // Outer loop controller
    double k1,k2,k3 ;
    k1 = 4.5;   // 6.5 for straight line 
    k2 = 4.5; // 6.5 for straight line 
    k3 = 8.5; // 6.5 for straight line
    double x_e ;
    double y_e ;
    double theta_e ;    
    x_e     =  cos(theta)*(xref - x) + sin(theta)*(yref - y) ;
    y_e     = -sin(theta)*(xref - x) + cos(theta)*(yref - y) ;
    theta_e =  thetaref - theta ; 
    vd    = vref*cos(theta_e) + k1*x_e ;
    wd    = wref + k3*signum(vref)*y_e + k2*theta_e ;

    // Inner loop controller
    double kp_v,ki_v ;
    double kp_w,ki_w ;
    kp_v = 2.5 ; // 2.5 for straight line
    ki_v = 2.5 ; // 2.5 for straight line
    kp_w = 1 ;   // 1 for straight line
    ki_w = 1 ;   // 1 for straight line
    double v_e, v_e_sum, w_e, w_e_sum ;
    v_e   = vd - v ;
    w_e   = wd - w ; 
    v_e_sum += v_e*dt ;
    w_e_sum += w_e*dt ;
    double F, Tau, tau1, tau2 ;
    F   = kp_v*v_e ; //+ ki_v*v_e_sum  ;
    Tau = kp_w*w_e ; //+ ki_w*w_e_sum  ;
    tau1 = (rho/2)*(F + Tau/W) ;
    tau2 = (rho/2)*(F - Tau/W) ;
    tau << tau1, tau2 ;

    // Dynamic model
    M <<                 mt,                  0.0,  2*mw*d*sin(theta),   0.0,   0.0,
                          0.0,                 mt, -2*mw*d*cos(theta),   0.0,   0.0,
          2*mw*d*sin(theta), -2*mw*d*cos(theta),                 It,   0.0,   0.0,
                          0.0,                  0.0,                  0.0, Iwy,   0.0,
                          0.0,                  0.0,                  0.0,   0.0, Iwy;                      
    C <<  0.0, 0.0, 2*mw*d*cos(theta)*thetadot, 0.0, 0.0,
          0.0, 0.0, 2*mw*d*sin(theta)*thetadot, 0.0, 0.0,
          0.0, 0.0,                        0.0, 0.0, 0.0,
          0.0, 0.0,                        0.0, 0.0, 0.0,
          0.0, 0.0,                        0.0, 0.0, 0.0;    
    B << 0.0, 0.0,
         0.0, 0.0,
         0.0, 0.0,
         1.0, 0.0,
         0.0, 1.0; 
    A <<  cos(theta), sin(theta),  W, -rho,    0.0,
          cos(theta), sin(theta), -W,    0.0, -rho,
         -sin(theta), cos(theta), -d,    0.0,    0.0;
    S << 0.5*rho*cos(theta), 0.5*rho*cos(theta), 
         0.5*rho*sin(theta), 0.5*rho*sin(theta),
                  0.5*rho/W,         -0.5*rho/W,
                        1.0,                0.0,
                        0.0,                1.0;                      
    Sdot << -0.5*rho*sin(theta)*thetadot, -0.5*rho*sin(theta)*thetadot, 
             0.5*rho*cos(theta)*thetadot,  0.5*rho*cos(theta)*thetadot,
                                   0.0,                        0.0,
                                   0.0,                        0.0, 
                                   0.0,                        0.0;                                                   
    Mhat = S.transpose()*M*S ;
    Vhat = S.transpose()*M*Sdot + S.transpose()*C*S ;
    Bhat = S.transpose()*B ;
    omegadot = Mhat.inverse()*(Bhat*tau - Vhat*omega) ;
    omega = omega + omegadot*dt ;
    v = 0.5*omega(0)*rho + 0.5*omega(1)*rho ;
    w = rho*(omega(0) - omega(1))/(2*W) ;
 
    // Kinematic model
    qdot = S*omega ;
    q = q + qdot*dt ;
    x = q(0) ;
    y = q(1) ;
    theta = q(2) ;
    thetadot = (theta - theta_prev)/dt ;
    phi1  = q(3) ;
    phi2  = q(4) ;
    theta_prev = theta ;
 
    ROS_INFO_STREAM("M =" << M);
    ROS_INFO_STREAM("C =" << C);
    ROS_INFO_STREAM("B =" << B);
    ROS_INFO_STREAM("A =" << A);
    ROS_INFO_STREAM("S =" << S);
    ROS_INFO_STREAM("Sdot =" << Sdot) ;

    // Debug
    debug_msg.linear.x =  vd       ;
    debug_msg.linear.y =  wd        ;
    debug_msg.linear.z =  v     ;
    debug_msg.angular.x = w     ;
    debug_msg.angular.y = 0      ;
    debug_msg.angular.z = 0      ;
    debug_pub.publish(debug_msg)   ;
    */
    
    
    visualization()           ;
    reference_trajectory(time, dt);
    outerloop_controller(dt)    ;
    innerloop_controller(dt)  ;
    dynamics_noslip(dt)       ;
    kinematics_noslip(dt)     ;
    debug() ;
    
    then = now;
    ros::spinOnce();
}

/*
void formation_control::update_dynamic_noslip()
{
    ros::Time now = ros::Time::now();

    double k1,k2,k3 ;
    k1 = 1;k2 = 6.5;k3 = 6.5;
    double kp,kd,ki ;
    kp = 1;kd = 1;ki = 10 ;

    double dt ;
    dt = 0.05 ; //now.toSec() - then.toSec(); 
    time = time + 0.05 ;
    ROS_INFO_STREAM("elapsed =" << dt);
        
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
}
*/

void formation_control::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
       update_dynamic_complex_noslip();
       //update_dynamic_noslip() ;
       visualization() ;
       loop_rate.sleep();
    }
}  
  

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "formation_control");

  formation_control formation_one ;

  formation_one.spin() ;
  
  return 0 ;
}
