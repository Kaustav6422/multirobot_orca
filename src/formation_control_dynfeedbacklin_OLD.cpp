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

class robot
{
public:
    robot()  ;
    void dynamics_noslip() ;
    void kinematics_noslip() ;
    void spin()            ;
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
    ros::Time then ;
};

robot::robot()
{
    init_variables() ;
}

/*
formation_control::formation_control()
{
    init_variables();

    // Publish
    cmd_vel_leader_pub    = nh.advertise<geometry_msgs::Twist>("/robot_0/mobile_base/commands/velocity",20) ;
    cmd_vel_pub_follower1 = nh.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",20);
    cmd_vel_pub_follower2 = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",20);
    debug_pub             = nh.advertise<geometry_msgs::Twist>("debug",20);
    marker_pub = n.advertise<visualization_msgs::Marker>("goal_trajectory", 20);  

    // Subscribe
    position_share_sub = nh.subscribe("position_share", 50, &formation_control::formation_callback, this);
    joy_sub            = nh.subscribe("robot_0/joy",50,&formation_control::joystick_callback,this) ;
}
*/

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
}

int robot::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
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
