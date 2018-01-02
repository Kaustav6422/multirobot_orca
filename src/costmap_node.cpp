#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <nav_core/base_global_planner.h>
#include <navfn/navfn_ros.h>

#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

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

using namespace std;
using std::string;

struct cells 
{
  int currentCell;
  float fCost;
};

namespace costmap_2d 
{
  class Costmap2DNode 
  {
    public:
      Costmap2DNode(tf::TransformListener& tf) : costmap_ros_("costmap", tf){}
    private:
      Costmap2DROS costmap_ros_;
  };
};


int main(int argc, char **argv)
{
    ros::init(argc,argv, "formation_control");

    tf::TransformListener tf(ros::Duration(10));
    //costmap_2d::Costmap2DROS* costmap_node;
    //costmap_node = new costmap_2d::Costmap2DROS("costmap",tf);
    //costmap_node->start() ;

    //costmap_2d::Costmap2DROS* costmap_ros_ ;
    //costmap_2d::Costmap2D* costmap_        ;
    //costmap_ = costmap_ros_->getCostmap() ;
    //double originX ;
    //double originY ;
    //originX = costmap_->getOriginX() ;
    //originY = costmap_->getOriginY() ;

    


    ////tf::TransformListener tf_(ros::Duration(10));
    //Costmap2DROS* localmap_ = new costmap_2d::Costmap2DROS(std::string("newlocal_costmap"), tf_);



    

    /*
    int width = costmap_->getSizeInCellsX();
    int height = costmap_->getSizeInCellsY();
    float resolution = costmap_->getResolution();
    int mapSize = width*height;
    float tBreak = 1+1/(mapSize); 
    int value =0;
    bool* OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        if (cost == 0)
          OGM[iy*width+ix]=true;
        else
          OGM[iy*width+ix]=false;
      }
    }
    */

    ros::NodeHandle nh ;

    // ros::Subscriber robotmaptf_sub =  nh.subscribe("position_share", 10, goal_callback);

    ros::spin() ;

    return 0 ;

} 

