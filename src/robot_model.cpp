#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
//#include <algorithm>
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
#include "Eigen/Dense"

using namespace std ;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class robot
{
public:
    robot()  ;
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
    void dynamics_noslip(double deltat) ;
    void kinematics_noslip(double deltat) ;
    void reference_trajectory(double time) ;
    void update()          ;
    void outerloop_controller() ;
    void innerloop_controller(double dt) ;
    void debug() ;

    geometry_msgs::Twist debug_msg      ;

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
    ros::Time then ;

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
    double vd,wd ;
    double v,w ;

    // Visualization variables
    double f ;

};

robot::robot()
{
    init_variables() ;

    marker_ref_pub        = nh.advertise<visualization_msgs::Marker>("trajectory_ref", 20);
    marker_robot_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_robot", 20) ; 
    debug_pub             = nh.advertise<geometry_msgs::Twist>("debug",20);
}

void robot::visualization()
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
}

void robot::init_variables()
{
    rate = 20 ;
    time = 0 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();

    // Dynamic properties (From Torres's thesis pg 45)
    mb = 1.5 ;     // kgs
    mw = 0.064 ;   // kgs
    mt = mb + 2*mw ;
    rho = 0.0365 ; // meters
    w = 0.105 ;    // meters
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
	B << 0, 0, 
	     0, 0, 
	     0, 0, 
	     0, 0, 
	     0, 0;

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
	phi1  = 0.0 ;
	phi2  = 0.0 ;
	q = VectorXd(5) ;
	q << x, y, theta, phi1, phi2 ;
	qdot = VectorXd(5) ;
    qdot << 0.0, 0.0, 0.0, 0.0, 0.0 ;

    f = 0.0 ;  
}

int robot::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

void robot::reference_trajectory(double time)
{
	double A,B ;
    A = 1.5 ;
    B = 0.5 ;
    xref = A*sin(B*time) ;
    yref = 2*A*sin(0.25*B*time) ;
    xref_dot = A*B*cos(B*time) ;
    yref_dot = 0.5*A*B*cos(0.25*B*time) ;
    thetaref = atan2(yref_dot,xref_dot) ;
    xref_ddot = -A*B*B*sin(B*time) ;
    yref_ddot = -0.5*0.25*A*B*B*sin(0.25*B*time) ;
    vref = sqrt(pow(xref_dot,2) + pow(yref_dot,2)) ;
    wref = (xref_ddot*yref_dot - yref_ddot*xref_dot)/pow(vref,2) ;
}

void robot::outerloop_controller()
{
	double k1,k2,k3 ;
    k1 = 1;k2 = 6.5;k3 = 6.5;

    double x_e ;
    double y_e ;
    double theta_e ;    

    x_e     =  cos(theta)*(xref - x) + sin(theta)*(yref - y) ;
    y_e     = -sin(theta)*(xref - x) + cos(theta)*(yref - y) ;
    theta_e =  thetaref - theta ; 

    vd    = vref*cos(theta_e) + k1*x_e ;
    wd    = wref + k3*signum(vref)*y_e + k2*theta_e ;
}

void robot::innerloop_controller(double dt)
{
	double kp,kd,ki ;
    kp = 1;kd = 1;ki = 10 ;

    double v_e, v_e_sum, w_e, w_e_sum ;
    v_e   = vd - v ;
    v_e_sum += v_e*dt ;
    w_e   = wd - w ; 
    w_e_sum += w_e*dt ;

    double F, Tau, tau1, tau2 ;
    F   = kp*v_e + ki*v_e_sum  ;
    Tau = kp*w_e + ki*w_e_sum  ;
    tau1 = (rho/2)*(F + Tau/W) ;
    tau2 = (rho/2)*(F - Tau/W) ;
    tau << tau1, tau2 ;
}

void robot::dynamics_noslip(double deltat)
{
	M <<                 mt,                  0,  2*mw*d*sin(theta),   0,   0,
	                      0,                 mt, -2*mw*d*cos(theta),   0,   0,
	      2*mw*d*sin(theta), -2*mw*d*cos(theta),                 It,   0,   0,
	                      0,                  0,                  0, Iwy,   0,
	                      0,                  0,                  0,   0, Iwy;

	C <<  0, 0, 2*mw*d*cos(theta)*pow(thetadot,2), 0, 0,
	      0, 0, 2*mw*d*sin(theta)*pow(thetadot,2), 0, 0,
	      0, 0,                                 0, 0, 0,
	      0, 0,                                 0, 0, 0,
	      0, 0,                                 0, 0, 0;

	B << 0, 0,
	     0, 0,
	     0, 0,
	     1, 0,
	     0, 1;

	A <<  cos(theta), sin(theta),  W, -rho,    0,
	      cos(theta), sin(theta), -W,    0, -rho,
	     -sin(theta), cos(theta), -d,    0,    0;

	S << 0.5*rho*cos(theta), 0.5*rho*sin(theta), 
	     0.5*rho*sin(theta), 0.5*rho*sin(theta),
	              0.5*rho/W,         -0.5*rho/W,
	                      1,                  0,
	                      0,                  1;

	Sdot << -0.5*sin(theta)*thetadot, -0.5*sin(theta)*thetadot, 
	         0.5*cos(theta)*thetadot,  0.5*cos(theta)*thetadot,
	                               0,                        0,
	                               0,                        0, 
	                               0,                        0;                    

	Mhat = S.transpose()*M*S ;
	Vhat = S.transpose()*M*Sdot + S.transpose()*C*S ;
	Bhat = S.transpose()*B ;
	omegadot = Mhat.inverse()*(Bhat*tau - Vhat) ;
	omega = omega + omegadot*deltat ;
	v = 0.5*omega(1)*rho + 0.5*omega(2)*rho ;
	w = rho*(omega(1) - omega(2))/(2*W) ;
}

void robot::kinematics_noslip(double deltat)
{
	qdot = S*omega ;
	q = q + qdot*deltat ;
	x = q(1) ;
	y = q(2) ;
	theta = q(3) ;
	thetadot = (theta - theta_prev)/deltat ;
	phi1  = q(4) ;
	phi2  = q(5) ;
	theta_prev = theta ;
}

void robot::debug()
{
	debug_msg.linear.x =  x         ;
    debug_msg.linear.y =  y         ;
    debug_msg.linear.z =  0         ;
    debug_msg.angular.x = xref      ;
    debug_msg.angular.y = yref      ;
    debug_msg.angular.z = 0         ;
    debug_pub.publish(debug_msg)    ;
}

void robot::update()
{
    ros::Time now = ros::Time::now();

	double dt ;
    dt = 0.05 ; //now.toSec() - then.toSec(); 
    time = time + 0.05 ;
    ROS_INFO_STREAM("elapsed =" << dt);

    reference_trajectory(time);
    outerloop_controller();
    innerloop_controller(dt) ;
	dynamics_noslip(dt);
	kinematics_noslip(dt);
	debug() ;
	visualization() ;

    then = now;
	ros::spinOnce();
}

void robot::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
       update() ;
       visualization() ;
       loop_rate.sleep() ;
    }
}  
  
int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "robot");

  robot robot_instance ;

  robot_instance.spin() ;

  return 0 ;
}

