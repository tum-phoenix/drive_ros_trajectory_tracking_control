#ifndef DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H
#define DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <drive_ros_msgs/TrajectoryMetaInput.h>
#include <drive_ros_msgs/Trajectory.h>
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>
#include <drive_ros_uavcan/phoenix_msgs__DriveState.h>

class TrajectoryTrackingController {
public:
    TrajectoryTrackingController(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~TrajectoryTrackingController();
protected:
    // actually just temp until we get a proper trajectory generator
    void metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg);
    void driveStateCB(const drive_ros_uavcan::phoenix_msgs__DriveStateConstPtr &msg);
    void processMetaInput();
    // this is where the control magic happens
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);

    // NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // subscribers
    ros::Subscriber meta_input_sub_;
    ros::Subscriber drive_state_sub_;
    ros::Subscriber trajectory_sub_;

    // publishers
    ros::Publisher nuc_command_pub_;

    // store current driving state
    float cur_v_ = 0;
    float cur_angle_f_ = 0;
    float cur_angle_r_ = 0;

    // store current meta inputs
    // todo: handle this stuff in trajectory generator, this should not be needed in the controller
    float v_max_ = 0;
    int driving_command_ = 0;
};


#endif //DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H
