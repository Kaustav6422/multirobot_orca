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

/*
#include "IPlanarGridMap.h"
#include "PlanarGridBinaryMap.h"
#include "PlanarGridContainer.h"
#include "PlanarGridIndex.h"
#include "PlanarGridOccupancyMap.h"private: 
*/

#define MAX_LINEAR_VEL 10.0
#define MAX_ANGULAR_VEL 10.0 

class robot
{
public:
    robot()  ;
    void dynamics_noslip(double tau1, double tau2) ;
    void kinematics_noslip(double v, double omega) ;
    void spin()            ;
    void visualization() ;
    int signum(double val) ;
    double time ;
private: 
    ros::NodeHandle nh ;
    ros::Publisher cmd_vel_pub_follower1 ;
    ros::Publisher cmd_vel_pub_follower2 ;
    ros::Publisher cmd_vel_leader_pub ;
    ros::Publisher debug_pub ;
    ros::Publisher marker_pub ;
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

    double rate ;
    ros::Time t_next ;
    ros::Duration t_delta ;
    double elapsed ;
    ros::Time then;

    // Dynamic properties
    double mb ;
    double mw ;
    double mt ;
    double rho ;
    double w ;
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

    // Kinematic properties
    double x ;
    double y ;
    double theta ;
    double thetadot ;

};

robot::robot()
{
    init_variables() ;

    /*
    // Publish
    cmd_vel_leader_pub    = nh.advertise<geometry_msgs::Twist>("/robot_0/mobile_base/commands/velocity",20) ;
    cmd_vel_pub_follower1 = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",20);
    cmd_vel_pub_follower2 = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",20);
    debug_pub             = nh.advertise<geometry_msgs::Twist>("debug",20);
    marker_pub = n.advertise<visualization_msgs::Marker>("goal_trajectory", 20);  

    // Subscribe
    position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
    joy_sub            = nh.subscribe("robot_0/joy",50,&formation_control::joystick_callback,this) ;
    */
}

void robot::visualization()
{

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
    It = 0 ;
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

	Mhat = S.transpose()*M*S ;
	Vhat = S.transpose()*M*Sdot + S.transpose()*C*S ;
	Bhat = S.transpose()*B ;              
	     
}

int robot::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

void robot::dynamics_noslip(double tau1, double tau2)
{
	M =                 mt,                  0,  2*mw*d*sin(theta),   0,   0,
	                     0,                 mt, -2*mw*d*cos(theta),   0,   0,
	     2*mw*d*sin(theta), -2*mw*d*cos(theta),                 It,   0,   0,
	                     0,                  0,                  0, Iwy,   0,
	                     0,                  0,                  0,   0, Iwy;

	C =  0, 0, 2*mw*d*cos(theta)*thetadot, 0, 0,
	     0, 0, 2*mw*d*sin(theta)*thetadot, 0, 0,
	     0, 0,                          0, 0, 0,
	     0, 0,                          0, 0, 0,
	     0, 0,                          0, 0, 0;

	B = 0, 0,
	    0, 0,
	    0, 0,
	    1, 0,
	    0, 1;

	A =  cos(theta), sin(theta),  w, -rho,    0,
	     cos(theta), sin(theta), -w,    0, -rho,
	    -sin(theta), cos(theta), -d,    0,    0;

	S = 0.5*rho*cos(theta), 0.5*rho*sin(theta), 
	    0.5*rho*sin(theta), 0.5*rho*sin(theta),
	             0.5*rho/w,         -0.5*rho/w,
	                     1,                  0,
	                     0,                  1;

	Sdot = -0.5*sin(theta)*thetadot, -0.5*sin(theta)*thetadot, 
	        0.5*cos(theta)*thetadot,  0.5*cos(theta)*thetadot,
	                              0,                        0,
	                              0,                        0, 
	                              0,                        0;                    

	Mhat = S.transpose()*M*S ;
	Vhat = S.transpose()*M*Sdot + S.transpose()*C*S ;
	Bhat = S.transpose()*B ;                    



}

void robot::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
       loop_rate.sleep();
    }
}  
  

int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "formation_control");

  robot robot_instance ;

  robot_instance.spin() ;

  return 0 ;
}
