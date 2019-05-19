//
// Created by sebastian on 19.05.19.
//

#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_control.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_tracking_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}