#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

void poseCallback(const sensor_msgs::joy & msg){
	
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_control_node");
    ros::NodeHandle nh;

    ros::Subscriber joySub = nh.subscribe("/joy", 10, //subscribe to topic
		                  joyCallback);

    // YOUR CODE HERE //

    // Don't change these lines
    ROS_INFO_STREAM("prizm_control_node ready!");
    ROS_INFO_STREAM("Don't forget to press the green button on the PRIZM!");
    ros::spin();
}
