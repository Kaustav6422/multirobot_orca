#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>

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

/*
#include "IPlanarGridMap.h"
#include "PlanarGridBinaryMap.h"
#include "PlanarGridContainer.h"
#include "PlanarGridIndex.h"
#include "PlanarGridOccupancyMap.h"
*/

// Movebase.cpp (computeVelocityCommands)
// baselocalplanner trajectory_planner_ros.h(computeVelocityCommands), trajectory.cpp , trajectory_planner.cpp(createTrajectories())

using namespace std ;

/*
template<typename T, typename S>
double pointsDistance(const T &one, const S &two)
{
      return sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
}
*/

/**
* @brief Evaluate whether two points are approximately adjacent, within a specified proximity distance.
* @param one Point one
* @param two Point two
* @param proximity Proximity distance
* @return True if approximately adjacent, false otherwise
**/

/*
template<typename T, typename S>
bool pointsNearby(const T &one, const S &two, const double &proximity)
{
      return pointsDistance(one, two) <= proximity;
}

int convertToCellIndex(float x, float y)
{

  int cellIndex;
  float newX = x / resolution;
  float newY = y / resolution;

  cellIndex = getCellIndex(newY, newX);

  return cellIndex;
}

int getCellIndex(int i,int j) //get the index of the cell to be used in Path
{
  return (i*width)+j;  
}
*/

/*
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void goal_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data)
{
    string robot_movebase = "robot_1/move_base"                             ;
    MoveBaseClient ac(robot_movebase.c_str(),true)                          ; 
    //while(!ac.waitForServer(ros::Duration(1.0)))
    while(!ac.waitForServer())  
    {
        ROS_INFO("Waiting for the move_base action server to come up")      ;
        ROS_INFO("%s",robot_movebase.c_str())                               ;
    }
    move_base_msgs::MoveBaseGoal goal                                       ; // to be used by move_base to send the goal 
    move_base_msgs::MoveBaseGoal current_goal                               ; // making a copy of current goal position

    if ((data->robot_id == "robot_0") and !pointsNearby(current_goal.target_pose.pose.position,data->pose.pose.position,0.05))  
    {
        tf::Quaternion q(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        goal.target_pose.header.frame_id    = "map"                             ; 
        goal.target_pose.header.stamp       = ros::Time::now()                  ;
        goal.target_pose.pose.position.x    = data->pose.pose.position.x + 1*cos(0.785)       ; 
        goal.target_pose.pose.position.y    = data->pose.pose.position.y + 1*sin(0.785)       ; 
        goal.target_pose.pose.position.z    = 0.0                                             ; 
        goal.target_pose.pose.orientation   = data->pose.pose.orientation   ;
        ROS_INFO("Sending goal")                                            ;
        ac.sendGoal(goal)                                                   ; 
        current_goal = goal                                                 ; // making a copy of current goal position 
        // ac.waitForResult()                                                  ;
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved to it's goal")                 ;
        else
            ROS_INFO("The base not reached it's goal yet")                ;

        //ac.cancelGoalsAtAndBeforeTime(ros::Time::now()) ;
    }      
}
*/


// Global variables

void formation_callback(const collvoid_msgs::PoseTwistWithCovariance::ConstPtr& data)
{

  ros::NodeHandle nh_cb ;

  double robot0_x ;
  double robot0_y ;
  double robot0_yaw ;
  double robot1_x ;
  double robot1_y ;
  double robot1_yaw ;

  ros::Publisher cmd_vel_pub = nh_cb.advertise<geometry_msgs::Twist>("/robot_1/mobile_base/commands/velocity",10);

  if (data->robot_id == "robot_0")
  { 
    robot0_x = data->pose.pose.position.x ;
    robot0_y = data->pose.pose.position.y ;
    tf::Quaternion q_robot0(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
    tf::Matrix3x3 m_robot0(q_robot0);
    double roll, pitch, yaw;
    m_robot0.getRPY(roll, pitch, yaw);
    robot0_yaw = yaw ;
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
  }

  double dist = sqrt(pow((robot0_x-robot1_x),2) + pow((robot0_y-robot1_y),2) )   ; //               sqrt((robot0_x-robot1_x)^2 + (robot0_y-robot1_y)^2) ;
  double MIN_DIST = 5 ;
  double MAX_LINEAR_VEL = 2 ;
  double MAX_ANGULAR_VEL = 2 ;
  double dist_prev   ;
 
  double error_dist = dist - MIN_DIST ;
  double error_dist_rate = dist - dist_prev ;

  geometry_msgs::Twist vel_msg ;
  if (dist > MIN_DIST)
  {
      vel_msg.linear.x  = min(1.5 * error_dist + 0.5*error_dist_rate, MAX_LINEAR_VEL)                           ;  
      vel_msg.angular.z = min(4 * atan2((robot0_y-robot1_y),(robot0_x-robot1_x)), MAX_ANGULAR_VEL) ;
  }
  cmd_vel_pub.publish(vel_msg) ;

  dist_prev = dist ;
}

void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
   std_msgs::Header header = msg->header;
   nav_msgs::MapMetaData info = msg->info;
   int width = info.width ;
   int height = info.height ;
   float resolution = info.resolution ;
   int mapSize = width*height ;
   float tbreak = 1+1/(mapSize); 
   bool* OGM = new bool [mapSize] ;
     

   for (unsigned int iy = 0; iy < width; iy++)
    {
      for (unsigned int ix = 0; ix < height; ix++)
      {
        //unsigned int costmap = msg->data) ; //static_cast<int>(costmap_->getCost(ix, iy));
        //cout<<cost;
        if (msg->data[iy*width+ix] != -1 && msg->data[iy*width+ix] >=99 ) //    (costmap(ix,iy) == 0)
        {
          OGM[iy*width+ix]=true;
        }
        else
        {
          OGM[iy*width+ix]=false;
        }
      }
    }

   ROS_INFO("Got map %d %d", info.width, info.height);
}


// /robot_0/move_base/local_costmap/costmap [nav_msgs/OccupancyGrid]

/*

void costmapCb(const nav_msgs::OccupancyGrid& map)
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    if (params_.map_frame_id_ != map.header.frame_id)
    {
        ROS_ERROR("Incoming costmap on topic %s has frame %s, expected %s. Costmap not updated.",
                  costmap_sub_.getTopic().c_str(),
                  map.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    // sparsify the costmap.
    // update instead of rewrite might be more efficient, but rewrite fast enough for now...
    PlanarPose map_origin(map.info.origin.position.x, map.info.origin.position.y, tf::getYaw(map.info.origin.orientation));
    trajectorycheck_map_ = boost::shared_ptr<PlanarGridBinaryMap>(new PlanarGridBinaryMap(map_origin, map.info.resolution));

    for (unsigned int h = 0; h < map.info.height; ++h)
    {
        const unsigned int h_offset = h * map.info.width;
        for (unsigned int w = 0; w < map.info.width; ++w)
        {
            // in ROS costmap_2d, costmap_2d::INSCRIBED obstacle corresponds
            // to occupancy 99. We set our threshold there.
            if ( map.data[h_offset + w] != -1 && map.data[h_offset + w] >= 99 )
            {
                trajectorycheck_map_->setGridValue( PlanarGridIndex(w,h), true);
            }
        }
    }
}

void costmapUpdateCb(const map_msgs::OccupancyGridUpdate &map_update)
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    if (params_.map_frame_id_ != map_update.header.frame_id)
    {
        ROS_ERROR("Incoming costmap on topic %s has frame %s, expected %s. Costmap not updated.",
                  costmap_sub_.getTopic().c_str(),
                  map_update.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    for (unsigned int h = 0; h < map_update.height; ++h)
    {
        const unsigned int h_offset = h * map_update.width;
        for (unsigned int w = 0; w < map_update.width; ++w)
        {
            // in ROS costmap_2d, costmap_2d::INSCRIBED obstacle corresponds
            // to occupancy 99. We set our threshold there.
            if ( map_update.data[h_offset + w] != -1 && map_update.data[h_offset + w] >= 99 )
            {
                trajectorycheck_map_->setGridValue( PlanarGridIndex( map_update.x + w, map_update.y + h), true);
            }
        }
    }
}

*/

int main(int argc, char **argv)
{
    ros::init(argc,argv, "robot_gloalpose");
    ros::NodeHandle nh ;

    //ros::Subscriber robotmaptf_sub =  nh.subscribe("position_share", 10, goal_callback);
    ros::Subscriber formation_control_sub = nh.subscribe("position_share", 10, formation_callback);

    //ros::Subscriber costmap_sub    =  nh.subscribe("/robot_0/move_base/local_costmap/costmap", 1000, costmap_callback);
    //costmap_sub_        = nh_.subscribe("move_base/global_costmap/costmap", 10, costmapCb);
    //costmap_update_sub_ = nh_.subscribe("move_base/global_costmap/costmap_updates", 10, costmapUpdateCb);


    ros::spin() ;

    return 0 ;

} 

