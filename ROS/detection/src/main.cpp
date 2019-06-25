#include <iostream>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include "detector.h"


using namespace sensor_msgs;
using namespace cv;
using namespace std;

// Global variables
detector::Detector detector_inst;
ros::Subscriber image_sub; // Receive image data
ros::Publisher pos_pub; // Publish target position data
string Config_File;


void image_callback(const ImageConstPtr& msg)  
{
    // ROS_INFO("Sequence: %d", msg->header.seq);

    // Convert to cv::Mat
    cv_bridge::CvImagePtr cv_ptr_frame;
    Mat image;
    cv_ptr_frame = cv_bridge::toCvCopy(msg, "bgr8");
    image = cv_ptr_frame->image;

    // Detection
    vector<int> ids;
    vector<Point2f> centers;
    detector_inst.detect(image, ids, centers);

    // Show result
    detector_inst.drawmarker(image, Scalar(0, 255, 0), 10);
    circle(image, Point2f(image.cols / 2, image.rows / 2), 5, Scalar(0, 0, 255), 5);
    namedWindow("image", 0);
    imshow("image", image);
    waitKey(1);

    // Publish target position data
    geometry_msgs::Pose2D pos;
    if(centers.size() > 0) // 检查到目标
    {
        pos.x = centers[0].x;
        pos.y = centers[0].y;
        pos.theta = 1; // vaild
    }
    else // 未检查到目标
    {
        pos.x = 0;
        pos.y = 0;
        pos.theta = 0; // invaild       
    }
    pos_pub.publish(pos);

    // ROS_INFO("pos: %f, %f, %d", pos.x, pos.y, (int)pos.theta);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Parameterss
    nh_priv.param<string>("Config_File", Config_File, "/home/pro/Documents/yansai_robot/src/detection/params/detector_params.yml"); // 配置文件

    // Initial detector
    if(!detector_inst.init(Config_File))
    {
        ROS_ERROR("Failed to initial detecter");
        return -1;
    }

    // Create publisher
    pos_pub = nh.advertise<geometry_msgs::Pose2D>("position", 10);

    // Subscribe image topic
    image_sub = nh.subscribe("frame", 1, &image_callback);

    ros::spin();
}


