//Sriparna Sengupta
//Homework Six

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

void joyCallback(const sensor_msgs::Joy & msg){
	ros::NodeHandle n;
	std_msgs::String topicName;
	n.getParam("topicName", topic_name);
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>(topicName, 1000);

	while(ros::ok())	{
		ROS_INFO_STREAM(msg);
		geometry_msgs::Twist t;
		t.linear.x = sqrt(pow(msg.axes[0], 2) + pow(msg.axes[1], 2)); //calculate velocity 
									      //based on magnitude
		if(msg.axes[1] < 0)	{
			t.linear.x = -t.linear.x;
		}
		t.angular.z = sqrt(pow(msg.axes[2], 2) + pow(msg.axes[3], 2));;
		if((msg.axes[2] < 0 || msg.axes[3] < 0) && (msg.axes[2] < 0 && msg.axes[3] < 0) == false)	{
			t.angular.z = -t.angular.z;
		}
		velocity_publisher.publish(t);
		ros::spin();
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_control_node");
    ros::NodeHandle nh;

    ros::Subscriber joySub = nh.subscribe("/joy", 10, //subscribe to topic
		                  joyCallback);

    // Don't change these lines
    ROS_INFO_STREAM("prizm_control_node ready!");
    ROS_INFO_STREAM("Don't forget to press the green button on the PRIZM!");
    ros::spin();
}
