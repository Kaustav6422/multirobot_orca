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

class dynamic_model
{
	public:
		dynamic_model() ;
		void spin() ;
		int signum(double val) ;
		void visualization() ;
		double time ;
	private:
		ros::NodeHandle nh ;
		void update() ;
		void init_variables() ;
		ros::Publisher marker_pub ;
		ros::Publisher debug_pub ;
		geometry_msgs::Twist debug_msg      ;

		double rate ;
		double dt   ;
        ros::Time t_next ;
        ros::Duration t_delta ;
        ros::Time then;

		double mass ;
		double inertia ;
		double F ;
		double Tau ;

		double x, x_p ;
		double y, y_p ;
		double phi, phi_p ;

        double xd,yd,phid ;
		double x_e,y_e,phi_e ;
		
		double v, v_p ;
		double w, w_p ;
		double vref, wref ;
		double vd, wd ;

		double v_e, w_e, v_e_p, w_e_p ;

		double k1,k2,k3 ;
		double kp,kd ;

        double f ;
};

int dynamic_model::signum(double val)
{
    if (val > 0.0) return  1 ;
    if (val < 0.0) return -1 ;
    return 0;
}

dynamic_model::dynamic_model()
{
	init_variables() ;
	marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory", 20);
	debug_pub  = nh.advertise<geometry_msgs::Twist>("debug",20);  
}

void dynamic_model::init_variables()
{
	rate = 20 ;
    time = 0  ;
    //dt   = 0.05 ;

    mass    = 0.1 ;
    inertia = 0.1 ;
    //F = 0 ;
    //Tau = 0 ;

    x = 0; y = 0; phi = 0 ;
    phid = 0 ;

    k1 = 1 ;
    k2 = 1 ;
    k3 = 1 ;
    kp = 1 ;
    kd = 1 ;

    t_delta = ros::Duration(1.0 / rate) ;
    t_next = ros::Time::now() + t_delta ; 
    then = ros::Time::now();

    f = 0.0 ;

}

void dynamic_model::visualization()
{
	visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= "/map";
    sphere_list.header.stamp= ros::Time::now();
    sphere_list.ns= "spheres";
    sphere_list.action= visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w= 1.0;
    sphere_list.id = 0;
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.1;
    sphere_list.scale.y = 0.1;
    sphere_list.scale.z = 0.1;
    // Points are green
    sphere_list.color.r = 1.0f;
    sphere_list.color.a = 1.0;

    geometry_msgs::Point p_robot1;
    p_robot1.x = x ;
    p_robot1.y = y ;
    p_robot1.z = 0 ;
    sphere_list.points.push_back(p_robot1);

    geometry_msgs::Point p_robot2;
    p_robot2.x = xd ;
    p_robot2.y = yd ;
    p_robot2.z = 0;
    sphere_list.points.push_back(p_robot2);

    marker_pub.publish(sphere_list);
    f += 0.04;
}

void dynamic_model::update()
{
	ros::Time now = ros::Time::now();

	if ( now > t_next) 
	{
		dt = now.toSec() - then.toSec(); 
        time = time + 0.05 ;
        ROS_INFO_STREAM("elapsed =" << dt);
        //ROS_INFO_STREAM("now =" << now.toSec());
        //ROS_INFO_STREAM("then =" << then.toSec());

        vref  =  0.1; //exp(-0.1*time) ;
        wref  =  0 ;  //-exp(-time)     ;
        xd = xd + vref*dt ;
        yd = 0 ; 

        x_e   =  cos(phi)*(xd - x) + sin(phi)*(yd - y) ;
        y_e   = -sin(phi)*(xd - x) + cos(phi)*(yd - y) ;
        phi_e =  phid - phi ; 

        vd    = vref*cos(phi_e) + k1*x_e ;
        wd    = wref + k3*signum(vref)*y_e + k2*phi_e ;

        v_e   = vd - v ;
        w_e   = wd - w ; 

        F   = kp*v_e + kd*(v_e - v_e_p)/dt ;
        Tau = kp*w_e + kd*(w_e - w_e_p)/dt ;

        v = v_p + (F/mass)*dt       ;
        w = w_p + (Tau/inertia)*dt  ;

        x   = x_p   + v*cos(phi)*dt ;
        y   = y_p   + v*sin(phi)*dt ;
        phi = phi_p + w*dt          ;

        x_p   = x   ;
        y_p   = y   ;
        phi_p = phi ;
        v_p   = v   ;
        w_p   = w   ;
        v_e_p = v_e ;
        w_e_p = w_e ;

        debug_msg.linear.x =  0        ;
        debug_msg.linear.y =  0        ;
        debug_msg.linear.z =  0            ;
        debug_msg.angular.x = x_e               ;
        debug_msg.angular.y = y_e                   ;
        debug_msg.angular.z = phi_e                 ;
        debug_pub.publish(debug_msg)                   ;

        then = now;
        ros::spinOnce();
	}
	 else 
    {
        ROS_INFO_STREAM("LOOP MISSED");
    } 
}

void dynamic_model::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
       ROS_INFO_STREAM("spin") ;
       update();
       visualization() ;
       loop_rate.sleep();
    }
}  

int main(int argc, char **argv)
{ 
  //Initiate ROS
  ros::init(argc, argv, "dynamic_model");

  // Create an object of class formation_control that will take care of everything
  dynamic_model instance ;

  instance.spin() ;
 
  return 0 ;
}







