// Simple Go stop - CJ Chung
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

ros::Publisher twistPub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gostop_node");
    ros::NodeHandle nh;
    
    twistPub = nh.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);

    geometry_msgs::Twist twistMsg;
    ros::Rate r(20);  // 1000/20 = 50 ms

    ROS_INFO_STREAM("Go for 2 seconds!!!!!!");
    for(int i=0; i<20*2; i++){  // 2 seconds
      twistMsg.linear.x = 2.0;
      twistPub.publish(twistMsg);
      r.sleep();
    }

    twistMsg.linear.x = 0.0;
    twistPub.publish(twistMsg);
    ROS_INFO_STREAM("STOP!!!!!!!!!!!!!!!!!!");
    
    ros::spin();
}
