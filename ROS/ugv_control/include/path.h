#ifndef PATH_H
#define PATH_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

namespace path{

class Path
{
public:
  // Interface function
  bool setup(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
  void spin();

private:
  void OdomCallback(const nav_msgs::Odometry& msg);
  void PoseCallback(const geometry_msgs::PoseStamped& msg);

private:
  // Objects
  ros::Subscriber odom_sub;
  ros::Subscriber pose_sub;
  ros::Publisher path_pub;

  tf::TransformListener listener;
  nav_msgs::Path path;

  // Parameters
  string fixed_frame;
  string odom_frame;
  bool use_odom;
};

}

#endif // PATH_H
