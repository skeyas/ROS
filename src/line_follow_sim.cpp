//Sriparna Sengupta
//Homework Nine

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <prizm_control/LineFollowConfig.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#define CVWIN_PREVIEW "threshold preview"

const double PI = 3.14159265359;

class LineFollow
{
public:
    LineFollow();
    ~LineFollow();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCallback(prizm_control::LineFollowConfig &config, uint32_t level);

private:
    int regionToTurn(const cv::Mat&image);
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_;

    dynamic_reconfigure::Server<prizm_control::LineFollowConfig> server_;

    int thresh_value_;
    double paper_detected_;

    double lowHue, highHue, lowSat, highSat, lowLum, highLum;

};

LineFollow::LineFollow()
    :nh_{"~"}, it_{nh_}
{
    // Subscribe to the camera publisher node
    image_sub_ = it_.subscribe("/car/camera1/image_raw", 1, &LineFollow::imageCb, this);
	ros::NodeHandle n;
    // Publish on the twist command topic
	pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_commander/line_follow/twist_cmd", 10);

	geometry_msgs::Twist twistMsg;

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&LineFollow::configCallback, this, _1, _2));

    // Default values
    thresh_value_ = 180;
	lowHue = 180;
	highHue = 255;
	lowSat = 90;
}

LineFollow::~LineFollow()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}

void LineFollow::configCallback(prizm_control::LineFollowConfig &config, uint32_t level)
{
    thresh_value_ = config.thresh;
	lowHue = config.mask_l_hue;
	highHue = config.mask_h_hue;
	lowSat = config.mask_l_sat;
	highSat = config.mask_h_sat;
	lowLum = config.mask_l_lum;
	highLum = config.mask_h_lum;
}

void LineFollow::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //Convert to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    constexpr int thresh_type = 0; // Normal
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat image_thresh;

    cv::inRange(hsv_image, cv::Scalar(lowHue/2, lowSat, lowLum), cv::Scalar(highHue/2, highSat, highLum), image_thresh);

	geometry_msgs::Twist t;

    // Call the function to determine which region has the most white pixels
	int directionToTurn = regionToTurn(image_thresh);

	//turn left
	if(directionToTurn == 0)	{
		t.linear.x = 0.25;
		t.angular.z = 2;
		pub_.publish(t);

	}

	//go forward
	if(directionToTurn == 1)	{
		t.linear.x = 2;
		pub_.publish(t);
	}

	//turn right
	if(directionToTurn == 2)	{
		t.linear.x = 0.25;
		t.angular.z = -2;
		pub_.publish(t);
	}

	//stop if line no longer visible

	if(directionToTurn == 3)	{
		t.linear.x = 0;
		t.angular.z = 0;
		pub_.publish(t);
	}

    // Show preview window
    cv::imshow(CVWIN_PREVIEW, image_thresh);

    // Update GUI Window
    cv::waitKey(3);

}

int LineFollow::regionToTurn(const cv::Mat&image)	{

	//divide image into sections
	cv::Mat r, l, c;
	image(cv::Rect(0, 0, image.cols / 3, image.rows)).copyTo(l);
    image(cv::Rect(image.cols / 3, 0, (image.cols / 3), image.rows)).copyTo(c);
    image(cv::Rect((image.cols / 3) * 2, 0, image.cols / 3, image.rows)).copyTo(r);


	//count white values in each section
	int left = cv::countNonZero(l);
	int right = cv::countNonZero(r);
	int forward = cv::countNonZero(c);

	if(left > right && left > forward)	{
		return 0;
	}

	else if(forward > left && forward > right)	{
		return 1;
	}

	else if(right > left && right > forward)	{
		return 2;
	}
	else	{
		return 3;
	}

}


int main(int argc, char** argv)	{
	ros::init(argc, argv, "line_follow_sim");
	LineFollow sd{};
	ROS_INFO_STREAM("line_follow running!");
    ros::spin();
}
