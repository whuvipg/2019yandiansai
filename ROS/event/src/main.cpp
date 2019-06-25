#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>

#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>

#include <signal.h>

#define FPN_PATH    "../Samples/config/FPN_1.txt"

using namespace cv;
using namespace std;

// void handler(int);

Mat imgevent;
void event(const sensor_msgs::Image &msg_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_img, "bgr8");
    imgevent = cv_ptr->image;
    imshow("infrared", imgevent);
    waitKey(1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "event");
	ros::NodeHandle n; 
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start read the event!\n");

    string SaveFolder;
    bool isShow;
    bool isSave;
	bool isSaveRaw;
    nh_priv.param<string>("SaveFolder", SaveFolder, "/home/pro/data/nonce/"); // 图片保存路径
    nh_priv.param<bool>("isShow", isShow, true);  // 是否显示图像
    nh_priv.param<bool>("isSave", isSave, false);  // 是否保存图像
    nh_priv.param<bool>("isSaveRaw", isSaveRaw, false);  // 是否原始数据	

	ros::Rate loop_rate(30);

	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("event", 1);
	sensor_msgs::ImagePtr msg;
  
	CeleX5 *pCeleX = new CeleX5;
	if (pCeleX == NULL)
		return 0;
	pCeleX->openSensor(CeleX5::CeleX5_MIPI);
	pCeleX->setFpnFile(FPN_PATH);

	CeleX5::CeleX5Mode sensorMode = CeleX5::Event_Address_Only_Mode;//Event_Address_Only_Mode  Event_Intensity_Mode
	pCeleX->setSensorFixedMode(sensorMode);
	pCeleX->setClockRate(10);

    char filename[128];
    int count = 0;
 
	while (ros::ok())
	{
		if (!pCeleX->getEventPicMat(CeleX5::EventBinaryPic).empty())
		{
			count++;

			cv::Mat src_img, filtered_img, img;
			src_img = pCeleX->getEventPicMat(CeleX5::EventBinaryPic);
			cv::medianBlur(src_img, filtered_img, 3); // 中值滤波
        	flip(filtered_img, img, 1); // 沿y轴翻转

			msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
			msg->header.stamp = ros::Time::now();  
			pub.publish(msg);

			if( isShow ) // 显示图像
			{
				imshow("event", img);
				waitKey(1);
			}

			if( isSave ) // 保存图像
			{
				sprintf(filename, "%05d.jpg", count);
				std::string str0(filename);
				imwrite(SaveFolder + "event/" + str0, img); 
			}

			if ( isSaveRaw ) // 保存原始数据
			{
				// record raw data
				std::vector<EventData> vecEvent;
				pCeleX->getEventDataVector(vecEvent);
				sprintf(filename, "%05d.txt", count);
				std::string str1(filename);
				ofstream out(SaveFolder + "raw_event/" + str1);
				for (int i = 0 ; i < vecEvent.size() ; i++)
				{
					out << vecEvent[i].t   << "\t" 
						<< vecEvent[i].row << "\t" 
						<< vecEvent[i].col << "\t" << endl; 
				}
				out.close();
			}

			// ROS_INFO( "read the %dth event successfully!", count );
		}

		loop_rate.sleep();
	}

	return 1;

}

// void handler(int sig)
// {
// 	// cout << "Recording ended." << endl;
// 	pCeleX->stopRecording();
// 	cout << "Recording ended1234" << endl;
// 	exit(0);
// }