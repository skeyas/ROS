//Sriparna Sengupta
//Homework Seven

// NOTE: Simply uncomment all lines with a leading "//%"
//% // Uncomment me!

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <prizm_control/StopOnWhiteConfig.h>

// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#define CVWIN_PREVIEW "threshold preview"

/**
 * Stop on White
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class StopOnWhite
{
public:
    StopOnWhite();
    ~StopOnWhite();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCallback(prizm_control::StopOnWhiteConfig &config, uint32_t level);

private:
    float whiteAmount(const cv::Mat& image);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_;

    dynamic_reconfigure::Server<prizm_control::StopOnWhiteConfig> server_;

    int thresh_value_;
    double paper_detected_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the StopOnWhite object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the image subscriber
 */
StopOnWhite::StopOnWhite()
    :nh_{"~"}, it_{nh_}
{
    // Subscribe to the camera publisher node
    image_sub_ = it_.subscribe("/cam_pub/image_raw", 1, &StopOnWhite::imageCb, this);
	ros::NodeHandle n;
    // Publish on the l2bot twist command topic
	pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 1000);

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&StopOnWhite::configCallback, this, _1, _2));

    // Default values
    thresh_value_ = 180;
    paper_detected_ = 0.3;
}



/**
 * Destructor
 * ==========
 *
 * Destroy CV windows
 */
StopOnWhite::~StopOnWhite()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}

/**
 * Dynamic Reconfigure Callback
 * ============================
 *
 * This function is called every time the Dynamic Reconfigure UI
 *   is updated by the user.
 */
void StopOnWhite::configCallback(prizm_control::StopOnWhiteConfig &config, uint32_t level)
{
    thresh_value_ = config.thresh;
    paper_detected_ = config.paper_detected;
}




/**
 * Callback function
 * =================
 *
 * Called once every time a image is published on the topic this
 *   node is subscribed to. The image is passed to the function as
 *   a ImageConstPtr
 */
void StopOnWhite::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // Convert the source to grayscale
    cv::Mat image_gray;
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);

    // Threshold parameters
    constexpr int max_BINARY_value = 255;
    constexpr int thresh_type = 0; // Normal
    cv::Mat image_thresh;
    cv::threshold(image_gray, image_thresh, thresh_value_, max_BINARY_value, thresh_type);

    // Call the function to determine the ratio of white pixels
    float white_amount = whiteAmount(image_thresh);

    // Uncomment to print pixel ratio to terminal
    ROS_INFO_STREAM("white/all pixel ratio: " << white_amount);

    // Publish robot movement command
    geometry_msgs::Twist twist;
    // If the number of white pixels is above a certain percent, stop
    // YOUR CODE HERE

    	twist.linear.x = 0.5;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
    	twist.angular.z = 0;	
    
    if(white_amount > paper_detected_)	{
    	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
    	twist.angular.z = 0;
    }
    pub_.publish(twist);

    // Show preview window
    cv::imshow(CVWIN_PREVIEW, image_thresh);

    // Update GUI Window
    cv::waitKey(3);

}


/**
 * Count White Pixels
 * ==========================
 * Return the ratio of white pixels
 * full white: 1.0
 * full black: 0.0
 *
 * https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html?highlight=threshold
 */
float StopOnWhite::whiteAmount(const cv::Mat& image)
{
	float total = image.rows * image.cols;
	float whiteAmount = cv::countNonZero(image);
	return (whiteAmount/total);
	
    // See note above, should return value between 0.0 and 1.0
    // YOUR CODE HERE
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_on_white");

    // Create a StopOnWhite object.
    // Since initilization code is in the constructor, we do
    //   not need to do anythong else with this object
    StopOnWhite sd{};

    ROS_INFO_STREAM("stop_on_white running!");
    ros::spin();
    return 0;
}
