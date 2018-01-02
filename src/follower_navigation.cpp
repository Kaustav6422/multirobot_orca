#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <algorithm>

#define MIN_DIST 0.8
#define MAX_LINEAR_VEL 1.5 
#define MAX_ANGULAR_VEL 3.14 

using namespace std ;

int main(int argc, char** argv)
{
  /*
  if(argc < 2)
  {
    ROS_ERROR("You must specify leader robot id.");
    return -1 ;
  }
  char *leader_id = argv[1] ;
  */

  ros::init(argc,argv,"follower");
  ros::NodeHandle nh ;

  string follower_id ;
  nh.getParam("follower_id", follower_id);
  string follower_frame = tf::resolve(follower_id, "base_link") ; 
  cout<< this_robot_frame << endl ;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);
  tf::TransformListener listener ;

  

  string leader_id ;
  nh.getParam("leader_id", leader_id);
  string leader_frame = tf::resolve(leader_id, "base_link") ; 
  cout<< leader_frame << endl ;

  listener.waitForTransform(follower_frame, leader_frame, ros::Time(0), ros::Duration(10.0)) ;
  ROS_INFO("%s is not following robot %s", follower_id.c_str(), leader_id) ;

  ros::Rate loopRate(10);

  float dist_from_leader_prev = 0 ;
  float error_dist = 0 ;
  float error_dist_rate = 0;
  float dist_from_leader = 0;

  while (ros::ok())
  {
    tf::StampedTransform transform ;
    
    try
    {
      listener.lookupTransform(follower_frame, leader_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    dist_from_leader = sqrt(pow(transform.getOrigin().x(),2) + pow(transform.getOrigin().y(),2));
   
    error_dist = dist_from_leader - MIN_DIST;

    error_dist_rate = dist_from_leader - dist_from_leader_prev ;
  
    geometry_msgs::Twist vel_msg ;

    if (dist_from_leader > MIN_DIST)
    {
      vel_msg.linear.x  = min(1.5 * error_dist + 0.5*error_dist_rate, MAX_LINEAR_VEL) ;  //min(0.5 * dist_from_leader, MAX_LINEAR_VEL) ;
      vel_msg.angular.z = min(4 * atan2(transform.getOrigin().y(), transform.getOrigin().x()), MAX_ANGULAR_VEL) ;
    }
    cmd_vel_pub.publish(vel_msg) ;

    dist_from_leader_prev = dist_from_leader ;
    loopRate.sleep() ;
  }

  return 0 ;
}
 
