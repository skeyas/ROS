//Sriparna Sengupta
//Homework Eleven

#include <ros/ros.h>
#include <route_publisher/Route.h>
#include <route_publisher/RouteCommand.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <vector>
#include <math.h>

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

double distance(const nav_msgs::Odometry& odom, const 
route_publisher::RouteCommand& goal)
{
    // Calculate the distance between the robot's current position (odom) and the goal

	geometry_msgs::Point p = odom.pose.pose.position;
	double d = getDistance(p.x, p.y, goal.x, goal.y);
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
class RouteFollower
{
public:
    RouteFollower();
    void routeCallback(const route_publisher::Route& route);
    void odomCallback(const nav_msgs::Odometry& odom);
    void pubGoal(double x, double y);
private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Subscriber route_sub_;
    ros::Subscriber odom_sub_;
    route_publisher::Route current_route_;
    int route_index_;
    bool is_executing_;
};
/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the RouteFollower object once and do nothing
 *   else (see main() below).
 *
 */
RouteFollower::RouteFollower() : nh_{"~"}
{
    // For publishing the goal
    goal_pub_ = 
nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/robot/move_base/goal", 10);
    // Route
    route_sub_ = nh_.subscribe("/route", 5, &RouteFollower::routeCallback, this);
    // Robot location
    odom_sub_ = nh_.subscribe("/robot/odom", 5, &RouteFollower::odomCallback, 
this);
    route_index_ = 0;
}
/**
 * Route Callback function
 * =======================
 *
 * Called when a new route is received
 */
void RouteFollower::routeCallback(const route_publisher::Route& route)
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
void RouteFollower::odomCallback(const nav_msgs::Odometry& odom)
{
    if (!is_executing_) return;
    double dist = distance(odom, current_route_.commands[route_index_]);
	if(dist < 0.7)	{
		route_index_++;
		pubGoal(current_route_.commands[route_index_].x, current_route_.commands[route_index_].y);
		if(route_index_ == current_route_.commands.size())	{
			is_executing_ = false;
			ROS_INFO_STREAM("Route complete");
		}
	}

    /* YOUR CODE HERE */
    // Use the distance to determine if we have arrived at the current coal
    // If we have, move to the next one and publish it
    // If we are done, set is_executing_ to false
}
void RouteFollower::pubGoal(double x, double y)
{
    // Goal Message
    auto msg = move_base_msgs::MoveBaseActionGoal{};
    // Required
    msg.header.frame_id = "map";
    msg.goal.target_pose.header.frame_id = "map";
    msg.goal.target_pose.pose.orientation.w = 1.0;

    // Set the position of the goal and publish it
	msg.goal.target_pose.pose.position.x = x;
	msg.goal.target_pose.pose.position.y = y;
	goal_pub_.publish(msg);

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "route_follower");
    // Create a RouteFollower object.
    // Since initilization code is in the constructor, we do
    //   not need to do anything else with this object
    RouteFollower rf{};
    ROS_INFO_STREAM("route_follower running!");
    ros::spin();
    return 0;
}
