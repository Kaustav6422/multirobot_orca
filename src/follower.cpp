#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
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

// tf::TransformListener listener ;
// listener.waitForTransform("/robot_0/base_link", "/map", ros::Time(0), ros::Duration(10.0));

using namespace std ;

class follower
{

public:
	follower();
	void spin();
        tf::TransformListener listener ;
private:
	ros::NodeHandle nh;

        ros::Time arduino_timestamp ;

        double goal_x ;
        double goal_y ;
        double goal_theta ;
        double rate ;

        // tf::TransformListener listener;

        ros::Publisher debug_pub ;
        

	void init_variables();
	void update();
};

follower::follower()
{
    init_variables();
    ROS_INFO("Started follower node");
    debug_pub = nh.advertise<geometry_msgs::Twist>("debug", 10);
    listener.waitForTransform("/robot_0/base_link", "/map", ros::Time(0), ros::Duration(10.0));
}

void follower::init_variables()
{
    goal_x = 0 ;
    goal_y = 0 ;
    goal_theta = 0 ;
    rate = 20 ; 
}

void follower::spin()
{
     ros::Rate loop_rate(rate);

     while (ros::ok())
	{
	    update();
	    loop_rate.sleep();
	}
}

void follower::update()
{
    try
    {
        listener.lookupTransform("/robot_0/base_link", "/map", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    goal_x = transform.getOrigin().x() ;
    goal_y = transform.getOrigin().y() ;
    goal_theta = getYaw(transform.getRotation()) ;  

    geometry_msgs::Twist debug_msg;
    debug_msg.linear.x = goal_x;
    debug_msg.linear.y = goal_y;
    debug_pub.publish(debug_msg); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"follower");

    //tf::TransformListener listener;

    follower obj;
    obj.spin();
    return 0;
}


