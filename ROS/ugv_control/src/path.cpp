#include "path.h"


namespace path{

bool Path::setup(ros::NodeHandle &nh, ros::NodeHandle &nh_priv){

    // Parameters
    nh_priv.param<string>("fixed_frame", fixed_frame, "map");
    nh_priv.param<string>("odom_frame", odom_frame, "odom");
    nh_priv.param<bool>("use_odom", use_odom, true);

    try{
        listener.waitForTransform(fixed_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ROS_ERROR("Initial TransformListener Failed");
        return false;
    }

    // Publish path message
    path_pub = nh.advertise<nav_msgs::Path>("trajectory", 10);
    path.header.stamp = ros::Time::now();
    path.header.frame_id = fixed_frame;

    // Subscribe topics
    if(use_odom)
    {
        odom_sub = nh.subscribe("odom", 10, &Path::OdomCallback, this);
    }
    else
    {
        pose_sub = nh.subscribe("pose", 10, &Path::PoseCallback, this);
    }

    return true;
}

void Path::spin(){

    ros::spin();
}

void Path::OdomCallback(const nav_msgs::Odometry& msg)
{
    // Receive Odometry message
    geometry_msgs::PoseStamped pose_stamped, path_pose;
    pose_stamped.header = msg.header;
    pose_stamped.pose = msg.pose.pose;

    // Point transform
    geometry_msgs::PointStamped odom_point, fixed_point;
    odom_point.header = pose_stamped.header;
    odom_point.header.stamp=ros::Time();
    odom_point.point.x = pose_stamped.pose.position.x;
    odom_point.point.y = pose_stamped.pose.position.y;
    odom_point.point.z = 0;

    try{
        listener.transformPoint(fixed_frame, odom_point, fixed_point);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    path_pose.header = fixed_point.header;
    pose_stamped.pose.position.x = fixed_point.point.x;
    pose_stamped.pose.position.y = fixed_point.point.y;
    pose_stamped.pose.position.z = 0;

    // Publish path
    path.poses.push_back(pose_stamped);
    path_pub.publish(path);
}

void Path::PoseCallback(const geometry_msgs::PoseStamped& msg)
{
    // Receive Odometry message
    geometry_msgs::PoseStamped pose_stamped, path_pose;
    pose_stamped = msg;

    // Point transform
    geometry_msgs::PointStamped odom_point, fixed_point;
    odom_point.header = pose_stamped.header;
    odom_point.header.stamp=ros::Time(); // 不能用 ros::Time::now()
    odom_point.point.x = pose_stamped.pose.position.x;
    odom_point.point.y = pose_stamped.pose.position.y;
    odom_point.point.z = 0;

    try{
        listener.transformPoint(fixed_frame, odom_point, fixed_point);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    path_pose.header = fixed_point.header;
    pose_stamped.pose.position.x = fixed_point.point.x;
    pose_stamped.pose.position.y = fixed_point.point.y;
    pose_stamped.pose.position.z = 0;

    // Publish path
    path.poses.push_back(pose_stamped);
    path_pub.publish(path);
}

};