# The `led_control` package

This is an in-class assignment. Please complete with your partner. Both partners must submit a copy of the code to their own repository.


## Install Dependencies using `rosdep`

The following command will install the joystick driver

```
$ cd <your_ros_ws>
$ rosdep install --from-paths src --ignore-src -r -y
```

## Instructions

Write a node that subscribes to the output of the joystick and publishes to the robot. You should be able to control the robot using the controller. Use the resources listed below to help you.

Additionally, the robot should automatically prevent the user from driving the robot into a wall using the IR distance sensor.

## Resources

[Joytick Driver](http://wiki.ros.org/joy)
[ROS Sensor Messages](http://wiki.ros.org/sensor_msgs)

## Packages you will need

https://github.com/LTU-AutoEV/prizm_ros
