#include <ros/ros.h>
#include "path.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Create Path class object
    path::Path path_inst;

    if(path_inst.setup(nh, nh_priv))
    {
        // Initial Path successfully
        path_inst.spin();
    }

    return 0;
}