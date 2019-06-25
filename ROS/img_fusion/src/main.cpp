#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>


using namespace sensor_msgs;
using namespace cv;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image> MySyncPolicy;

string SaveFolder;
bool isShow;
bool isSave;
char filename[128];
Mat imgframe;
Mat imginfrared;
Mat imgevent;


void imgfusion_callback(const ImageConstPtr& frame, const ImageConstPtr& infrared, const ImageConstPtr& event)  //回调中包含多个消息
{
    ROS_INFO("Sequence: %d", event->header.seq);

    // frame
    cv_bridge::CvImagePtr cv_ptr_frame;
    cv_ptr_frame = cv_bridge::toCvCopy(frame, "bgr8");
    imgframe = cv_ptr_frame->image;

    // infrarec
    cv_bridge::CvImagePtr cv_ptr_infrared;
    cv_ptr_infrared = cv_bridge::toCvCopy(infrared, "bgr8");
    imginfrared = cv_ptr_infrared->image;

    // event
    cv_bridge::CvImagePtr cv_ptr_event;
    cv_ptr_event = cv_bridge::toCvCopy(event, "bgr8");
    imgevent = cv_ptr_event->image;

    if ( isShow )
    {
        imshow("frame", imgframe);
        imshow("infrared", imginfrared);
        imshow("event", imgevent);
        waitKey(1);
    }

    if ( isSave )
    {
        sprintf(filename,"%05d.jpg", event->header.seq);
        std::string str(filename);
        imwrite(SaveFolder + "frame/" + str, imgframe);
        imwrite(SaveFolder + "infrared/" + str, imginfrared);
        imwrite(SaveFolder + "event/" + str, imgevent);
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"img_fusion");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start fuse the image!\n");  

    nh_priv.param<string>("SaveFolder", SaveFolder, "/home/pro/data/2.normallight/sky/"); // 图片保存路径
    nh_priv.param<bool>("isShow", isShow, true);  // 是否显示图像
    nh_priv.param<bool>("isSave", isSave, true);  // 是否保存图像



    message_filters::Subscriber<Image> frame_sub(nh, "frame", 1);           // topic1 输入
    message_filters::Subscriber<Image> infrared_sub(nh, "infrared", 1);     // topic2 输入
    message_filters::Subscriber<Image> event_sub(nh, "event", 1);           // topic3 输入

    // message_filters::TimeSynchronizer<Image, Image, Image> sync(frame_sub, infrared_sub, event_sub, 10); // 同步
    // sync.registerCallback(boost::bind(&imgfusion_callback, _1, _2, _3)); // 回调

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), frame_sub, infrared_sub, event_sub);
    sync.registerCallback(boost::bind(&imgfusion_callback, _1, _2, _3));

    ros::spin();
}


