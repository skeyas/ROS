# Author: Sriparna Sengupta

# Homework Five: Move Turtle With Joystick

This assignment consists of the files TPcontrol_joy_node.cpp and TP_joy.launch. It reads joystick data from the /joy topic and publishes geometry_msgs/Twist messages to /turtle1/cmd_vel to move the turtle.

# Homework Six:
This assignment consists of the files joy_control_node.cpp and TP_joy.launch. It reads joystick data from the /joy topic and publishes geometry_msgs/Twist messages to /prizm/twist_controller/twist_cmd to move the robot. The launch file is the same as for Homework Five, with the lines pertaining to the turtle commented out.

# Homework Seven
This assignment consists of the files stop_on_white.cpp and stop_on_white.launch. It publishes geometry_msgs/Twist messages to /prizm/twist_controller/twist_cmd to move the robot. The program takes the sensor_msgs/Image message from the camera and calculates the amount of white pixels visible. When that exceeds a given threshold, the robot stops.

# Homework Eight-One
This assignment consists of the files line_follow.cpp and line_follow.launch. It tracks which section of the image received by the camera contains the most blue pixels and outputs to the user which direction to turn based on that.

# Homework Nine
This assignment consists of the files line_follow_sim.cpp and line_follow_sim.launch. It tracks which section of the image from the Gazebo contains the most blue pixels and moves the simulated robot in that direction.

#Homework Ten
This assignment consists of the files move_to_object.cpp and move_to_object.launch. It moves the simulated robo in the direction of the nearest object tracked by the LIDAR.
