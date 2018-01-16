// DONE
void MePublisher::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    //we assume that the odometry is published in the frame of the base

    boost::mutex::scoped_lock(me_lock_);
    twist_.linear.x = msg->twist.twist.linear.x;
    twist_.linear.y = msg->twist.twist.linear.y;
    twist_.angular.z = msg->twist.twist.angular.z;

    last_seen_ = msg->header.stamp;

    if ((ros::Time::now() - last_time_me_published_).toSec() > publish_me_period_) 
    {
        last_time_me_published_ = ros::Time::now();

        publishMePoseTwist(); // publish to position_share 

    }
}

void MePublisher::publishMePoseTwist() 
{
    collvoid_msgs::PoseTwistWithCovariance me_msg;
    if (createMeMsg(me_msg, global_frame_)) 
    {
        position_share_pub_.publish(me_msg);
    }
}

bool MePublisher::createMeMsg(collvoid_msgs::PoseTwistWithCovariance &me_msg, std::string target_frame) // target_frame =  global_frame = map
{
    me_msg.header.stamp = ros::Time::now();
    me_msg.header.frame_id = target_frame;
    tf::Stamped <tf::Pose> global_pose;

    if (getGlobalPose(global_pose, target_frame, me_msg.header.stamp)) // passing empty global_pose, target_frame = map
    {
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        me_msg.pose.pose = pose_msg.pose;
    }
    else 
    {
        return false;
    }

    me_msg.twist.twist = twist_;

    me_msg.controlled = controlled_;
    me_msg.holonomic_velocity.x = holo_velocity_.x();
    me_msg.holonomic_velocity.y = holo_velocity_.y();

    me_msg.holo_robot = holo_robot_;
    me_msg.radius = (float)(uninflated_robot_radius_ + cur_loc_unc_radius_);
    me_msg.robot_id = my_id_;
    me_msg.controlled = controlled_;

    me_msg.footprint = createFootprintMsgFromVector2(minkowski_footprint_);

    return true;
}

bool MePublisher::getGlobalPose(tf::Stamped <tf::Pose> &global_pose, std::string target_frame, const ros::Time stamp) // target_frame =  global_frame = map
{
    //let's get the pose of the robot in the frame of the plan
    global_pose.setIdentity();
    global_pose.frame_id_ = base_frame_;
    global_pose.stamp_ = stamp;
    //global_pose.setRotation(tf::createIdentityQuaternion());
    try 
    {
        tf_->waitForTransform(target_frame, base_frame_, global_pose.stamp_, ros::Duration(0.2));
        tf_->transformPose(target_frame, global_pose, global_pose);
    }
    catch (tf::TransformException ex) {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        return false;
    };
    return true;
}//

void formation_control::odom_callback_leader(const nav_msgs::Odometry::ConstPtr& data) 
{
    collvoid_msgs::PoseTwistWithCovariance me_msg;

    tf::Stamped <tf::Pose> global_pose;

    global_pose.setIdentity();
    global_pose.frame_id_ = base_frame_;
    global_pose.stamp_ = ros::Time::now();

    try 
    {
        tf_->waitForTransform(global_frame, base_frame, global_pose.stamp_, ros::Duration(0.2));
        tf_->transformPose(global_frame, global_pose, global_pose);
    }
    catch (tf::TransformException ex) 
    {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        //return false;
    };
    geometry_msgs::PoseStamped pose_msg;
    tf::poseStampedTFToMsg(global_pose, pose_msg);









    
	robot0_x = data->pose.pose.position.x ;
	robot0_y = data->pose.pose.position.y ;
	tf::Quaternion q_robot0(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
    tf::Matrix3x3 m_robot0(q_robot0);
    double roll, pitch, yaw;
    m_robot0.getRPY(roll, pitch, yaw);
    robot0_yaw = yaw ;
    robot0_vx = data->twist.twist.linear.x ;
    robot0_xdot = robot0_vx*cos(robot0_yaw) ;
    robot0_ydot = robot0_vx*sin(robot0_yaw) ;
}

