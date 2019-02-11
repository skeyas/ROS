#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prizm_control_node");
    ros::NodeHandle nh;

    // YOUR CODE HERE //

    // Don't change these lines
    ROS_INFO_STREAM("prizm_control_node ready!");
    ROS_INFO_STREAM("Don't forget to press the green button on the PRIZM!");
    ros::spin();
}
