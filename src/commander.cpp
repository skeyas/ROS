/* Note: This started code is not complete and will not run
 * It should be used as a skeleton for the final project node
 */

#include <ros/ros.h>
#include <route_publisher/Route.h>
#include <route_publisher/RouteCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
// Any others...

#include <vector>
#include <math.h>

enum State {
    WAIT,
    LINE_FOLLOW,
    GOTO,
    SEARCH,
	AVOID
};

State convert(const std::string& str)
{
    if(str == "wait") return WAIT;
    else if(str == "line_follow") return LINE_FOLLOW;
    else if(str == "goto") return GOTO;
	else if(str == "search") return SEARCH;
	else if(str == "avoid_obstacle") return AVOID;
}

// Used for tracking if the robot has reached the next waypoint
double distance(const nav_msgs::Odometry& odom, const route_publisher::RouteCommand& goal)	{
	geometry_msgs::Point p = odom.pose.pose.position;
	double d = sqrt(pow((p.x-goal.x),2) + pow((p.y-goal.y),2));
	return d;
}


/**
 * Route Follower
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class RobotCommander
{
public:
    RobotCommander();

    // Route and odom callbacks
    void routeCallback(const route_publisher::Route& route);
    void odomCallback(const nav_msgs::Odometry& odom);

    // Callbacks for each of the nodes
    void lineFollowCallback(const geometry_msgs::Twist& twist);
    void gotoCallback(const geometry_msgs::Twist& twist);
    void findObstacleCallback(const geometry_msgs::Twist& twist);


    // Util Functions
    void pubGoal(double x, double y);

private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
	ros::Publisher final_twist_pub_;
    ros::Subscriber route_sub_;
    ros::Subscriber odom_sub_;
    // Subscribers for each node...

	ros::Subscriber line_follow_sub;
	ros::Subscriber go_to_waypoint_sub;
	ros::Subscriber go_to_obstacle_sub;
	

    route_publisher::Route current_route_;
    int route_index_;
    bool is_executing_;
    State current_state_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the RobotCommander object once and do nothing
 *   else (see main() below).
 *
 */
RobotCommander::RobotCommander() : nh_{"~"}
{
    // For publishing the goal
    // Topic specified in the given launch file
    goal_pub_ = nh_.advertise<geometry_msgs::Point>("/goto_node/goal", 10);

    // Route
    route_sub_ = nh_.subscribe("/route", 5, &RobotCommander::routeCallback, this);
    // Robot location
    odom_sub_ = nh_.subscribe("/car/odom", 5, &RobotCommander::odomCallback, this);


    // Subscribers for all other nodes
    // ...

	line_follow_sub = nh_.subscribe("/robot_commander/line_follow/twist_cmd", 5, &RobotCommander::lineFollowCallback, this);

	go_to_waypoint_sub = nh_.subscribe("/robot_commander/goto/twist_cmd", 5, &RobotCommander::gotoCallback, this);

	go_to_obstacle_sub = nh_.subscribe("/robot_commander/avoid_obstacle/twist_cmd", 5, &RobotCommander::findObstacleCallback, this);

    // Publisher for robot
     final_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);
	//goal_pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);

    route_index_ = 0;
}



/**
 * Route Callback function
 * =======================
 *
 * Called when a new route is received
 */
void RobotCommander::routeCallback(const route_publisher::Route& route)
{

    ROS_INFO_STREAM("Got new route!");
    if (route.commands.size() == 0)
    {
        ROS_ERROR_STREAM("Invalid empty route!");
        exit(1);
    }

    is_executing_ = true;
    current_route_ = route;
    route_index_ = 0;
	current_state_ = convert(current_route_.commands[route_index_].command);

    // Send initial goal
    pubGoal(current_route_.commands[route_index_].x,
            current_route_.commands[route_index_].y);
}



/**
 * Odom Callback function
 * =======================
 *
 * Called when the robot updates its position
 */
void RobotCommander::odomCallback(const nav_msgs::Odometry& odom)
{
    if (!is_executing_) return;

    if (distance(odom, current_route_.commands[route_index_]) < 1)
    {
        // Move to next command...
        // Set the current_state_ variable to the command
        // Publish goal if needed
		route_index_++;
		current_state_ = convert(current_route_.commands[route_index_].command);
		ROS_INFO_STREAM("at goal");
		ROS_INFO_STREAM(current_route_.commands[route_index_].command);
		if(current_state_ == GOTO)	{
			pubGoal(current_route_.commands[route_index_].x, current_route_.commands[route_index_].y);
		}
    }
}

void RobotCommander::pubGoal(double x, double y)
{
    // Publish a geometry_msgs::Point on the topic specified in the sim_course launch file
	geometry_msgs::Point msg;
	msg.x = x;
	msg.y = y;
    goal_pub_.publish(msg);
}


/**
 * Example callback function for one of the behavior nodes
 */
void RobotCommander::lineFollowCallback(const geometry_msgs::Twist& twist)
{
    // If we are not in the line follow state, ignore this message
    // Othwerise execute normally
	if (current_state_ != LINE_FOLLOW) return;
	final_twist_pub_.publish(twist);
}

void RobotCommander::gotoCallback(const geometry_msgs::Twist& twist)	{
	if(current_state_ != GOTO)	{
		return;
	}
	final_twist_pub_.publish(twist);
}

void RobotCommander::findObstacleCallback(const geometry_msgs::Twist& twist)	{
	if(current_state_ != SEARCH && current_state_ != AVOID)	{
		return;
	}
	if(current_state_ == SEARCH)	{
		final_twist_pub_.publish(twist);
	}
	if(current_state_ == AVOID)	{
		geometry_msgs::Twist t;
		t.linear.x = twist.linear.x;
		t.angular.z = -twist.angular.z;
		final_twist_pub_.publish(t);
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_commander");

    // Create a RobotCommander object.
    // Since initilization code is in the constructor, we do
    //   not need to do anything else with this object
    RobotCommander rf{};

    ROS_INFO_STREAM("robot_commander running!");
    ros::spin();
    return 0;
}
