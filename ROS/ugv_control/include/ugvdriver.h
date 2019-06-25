#ifndef UGVDRIVER_H
#define UGVDRIVER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include "serialtransmission.h"
#include "coordinatetransform.h"

using namespace std;

namespace ugv_driver{

class UgvDriver
{
public:
  // Interface function
  bool setup(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
  void spin();

private:
  // Velocity command callback.
  void VelCallback(const geometry_msgs::Twist& msg);
  // VO callback.
  void VOCallback(const geometry_msgs::Pose2D& msg);  

  void ProcessSensorData();
  // Publish sensor data
  void PublishSensorData();
  // Receive sensor data
  bool RecvSensorData();
  // Send velocity data
  bool SendVelData();

private:
  // Objects
  serial_transmission::SerialConfig serialconfig;
  serial_transmission::SerialTransmission serialtransmission;

  ros::Subscriber vel_sub; // subscribe velocity data
  ros::Subscriber pose_sub; // subscribe VO data 
  
  ros::Publisher odom_pub; // publish odometry data
  ros::Publisher imu_pub; // publish IMU data
  ros::Publisher gps_pub; // publish GPS data
  
  tf::TransformBroadcaster odom_broadcaster; // broadcast coordinate transform between odom frame and base_link frame

  geometry_msgs::Pose2D wheel_pose; // wheel odometry pose data 
  geometry_msgs::Pose2D vo_pose; // VO pose data
  geometry_msgs::Pose2D pose; // final pose data 
  nav_msgs::Odometry wheel_odom; // wheel odometry data
  nav_msgs::Odometry last_wheel_odom; // wheel odometry data
  geometry_msgs::Twist vel_twist;  // velocity data
  sensor_msgs::Imu imu;
  sensor_msgs::MagneticField magnetometer;
  ct::GlobalPosition globalpos, globalpos0;
  ct::LocalPosition localpos;

  // Parameters
  double roll, pitch, yaw, yaw0;
  bool isfirst;
  bool isvoinit;
  double vo_init_thresh;
  double vo_scale_x;
  double vo_scale_y;

  string base_frame;
  string odom_frame;
  bool use_vo;
  bool use_gps;
  bool pub_base_odom_transform;
  int odom_rate; // rate of publishing imu data
  int imu_accel_range; // IMU acceleration range
  int imu_gyro_range; // IMU gyroscope range
};

}

#endif // UGVDRIVER_H
