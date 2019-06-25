#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

using namespace std;

#define MAX_COUNT 5

// Global variables
ros::Subscriber pos_sub; // Receive target position data
ros::Publisher vel_pub; // Publish velocity data
geometry_msgs::Pose2D uav_pos;

int rate;
double height, width;
double k_x, k_theta; 
double min_v_x, max_v_x;
double min_v_th, max_v_th;
int cnt = 0;

void position_callback(const geometry_msgs::Pose2D& msg)  
{
    // ROS_INFO("position_callback");

    uav_pos = msg;
}

void TimerCallback(const ros::TimerEvent&)
{
	//ROS_INFO("TimerCallback");
    
    // Receive data
    double x = uav_pos.x;
    double y = uav_pos.y;
    bool vaild = int(uav_pos.theta) == 1;

    double v_x, v_th;
    if(vaild) // 检测到目标
    {
        cnt = 0;

        double dx = y - height / 2; // 图像的y轴为无人车的x轴
        double dy = width / 2 - x;  // 图像的负x轴为无人车的y轴

        // Calculate velocity
        v_x = k_x * dx; // 无人车只在x和theta上运动，因此 v_y = 0
        v_th = k_theta * dy;
    }
    else // 未检测到目标
    {
        if(cnt == MAX_COUNT) 
        {
            v_x = 0;
            v_th = 0;
        }
        else
        {
            cnt++;
        }
    }
    
    v_x = (abs(v_x) > max_v_x) ? ((v_x > 0) ? max_v_x : -max_v_x) : v_x;
    v_x = (abs(v_x) < min_v_x) ? 0 : v_x;
    v_th = (abs(v_th) > max_v_th) ? ((v_th > 0) ? max_v_th : -max_v_th) : v_th;
    v_th = (abs(v_th) < min_v_th) ? 0 : v_th;    

    // Publish velocity data
    geometry_msgs::Twist vel_twist;
    vel_twist.linear.x = v_x;
    vel_twist.linear.y = 0;
    vel_twist.angular.z = v_th;
    vel_pub.publish(vel_twist);

    // ROS_INFO("cmd_vel: %f, %f, %f", vel_twist.linear.x, vel_twist.linear.y, vel_twist.angular.z);
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Parameterss
    nh_priv.param<int>("rate", rate, 10);
    nh_priv.param<double>("height", height, 480);
    nh_priv.param<double>("width", width, 640);
    nh_priv.param<double>("k_x", k_x, 0.01);
    nh_priv.param<double>("k_theta", k_theta, 0.01); 
    nh_priv.param<double>("min_v_x", min_v_x, 0.1);       
    nh_priv.param<double>("max_v_x", max_v_x, 0.5);
    nh_priv.param<double>("min_v_th", min_v_th, 0.1);      
    nh_priv.param<double>("max_v_th", max_v_th, 0.5);    
    
    // Create publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Subscribe image topic
    pos_sub = nh.subscribe("uav_position", 10, &position_callback);

    // Create a timer
	ros::Timer timer = nh.createTimer(ros::Duration(1.0 / rate), &TimerCallback);

    ros::spin();
}


