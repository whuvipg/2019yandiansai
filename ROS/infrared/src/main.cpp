#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;
  
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "infrared");
    ros::NodeHandle n; 
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start read the infrared!\n");

    string SaveFolder;
    bool isShow;
    bool isSave;
    string serial_port;
    nh_priv.param<string>("SaveFolder", SaveFolder, "/home/pro/data/temp/"); // 图片保存路径
    nh_priv.param<bool>("isShow", isShow, false);  // 是否显示图像
    nh_priv.param<bool>("isSave", isSave, false);  // 是否保存图像
    nh_priv.param<string>("serial_port", serial_port, "/dev/ttyVideo0");  // 是否保存图像

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("infrared", 1);
    sensor_msgs::ImagePtr msg;

    VideoCapture cap(serial_port);
    if( !cap.isOpened() )
    {
        ROS_INFO("Read Infrared Video failed!\n");
        return 0;
    }

    Mat src_img, img;
    int count = 0;
    char imgname[128];
    while(ros::ok())
    {
        count++;

        cap >> src_img;
        if( src_img.empty() )
            break;
        flip(src_img, img, -1); // x,y均翻转
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = ros::Time::now();  
        pub.publish(msg);

        if( isShow ) // 显示图像
        {
            imshow("infrared", img);
            waitKey(1);
        }

        if( isSave ) // 保存图像
        {
            sprintf(imgname, "%05d.jpg", count);
            std::string str(imgname);
            imwrite(SaveFolder + "infrared/" + str, img); 
        }

        // ROS_INFO( "read the %dth infrared successfully!", count );
    }

    cap.release();

    return 0;
 

}
