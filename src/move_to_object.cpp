//Sriparna Sengupta
//Homework Ten

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <prizm_control/MoveToObjectConfig.h>

#include <vector>
#include <limits> // for infinity



/**
 * Find Obstacle
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class FindObstacle
{
public:
    FindObstacle();
    void scanCb(const sensor_msgs::LaserScan& msg);
    void configCallback(prizm_control::MoveToObjectConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber scan_sub_;

    dynamic_reconfigure::Server<prizm_control::MoveToObjectConfig> server_;
    prizm_control::MoveToObjectConfig config_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the FindObstacle object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the LaserScan subscriber
 */
FindObstacle::FindObstacle() : nh_{"~"}
{
    std::string scan_sub_topic;
    if (!nh_.getParam("scan_topic", scan_sub_topic))
    {
        ROS_ERROR_STREAM("Please set param 'scan_topic'");
        scan_sub_topic = "/lidar2d/scan";
    }
    else
    {
        ROS_INFO_STREAM("Using scan topic '" << scan_sub_topic << "'");
    }

    // Subscribe to the laser scan
    scan_sub_ = nh_.subscribe(scan_sub_topic, 5, &FindObstacle::scanCb, this);

    // Publish on the twist command topic
    pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_commander/avoid_obstacle/twist_cmd", 10);

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&FindObstacle::configCallback, this, _1, _2));

    // Load defaults
    server_.getConfigDefault(config_);
}



/**
 * Dynamic Reconfigure Callback
 * ============================
 *
 * This function is called every time the Dynamic Reconfigure UI
 *   is updated by the user.
 */
void FindObstacle::configCallback(prizm_control::MoveToObjectConfig &config, uint32_t level)
{
    config_ = config;
}


/**
 * Callback function
 * =================
 *
 * Called once every time a scan is published on the topic this
 *   node is subscribed to.
 */
void FindObstacle::scanCb(const sensor_msgs::LaserScan& msg)
{
    geometry_msgs::Twist twist;

    // **** YOUR CODE HERE ****
	
	double dist = msg.ranges[0];
	std::vector<float> distances = msg.ranges;
	double closest =  *min_element(distances.begin(), distances.end()); //distance of closest object
	int angle = std::distance(distances.begin(), min_element(distances.begin(), distances.end())); //angle of cloest object

	if(closest <= 0.5 || closest >= 9e99)	{
		twist.linear.x = 0;
		twist.angular.z = 0;
	} 
	else	{
		twist.linear.x = 1;
		if(angle > 90)	{
			twist.angular.z = 1;
		}
		if(angle < 90)	{
			twist.angular.z = -1;
		}
	}

    // Apply the turn multiplier
    twist.angular.z *= config_.steering_multip;
    // Publish the message
    pub_.publish(twist);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_to_obstacle");

    // Create a FindObstacle object.
    // Since initilization code is in the constructor, we do
    //   not need to do anything else with this object
    FindObstacle sd{};

    ROS_INFO_STREAM("go_to_obstacle running!");
    ros::spin();
    return 0;
}
