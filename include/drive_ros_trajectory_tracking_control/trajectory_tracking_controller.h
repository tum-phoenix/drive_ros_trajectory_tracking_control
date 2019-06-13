#ifndef DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H
#define DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H

#include <drive_ros_msgs/TrajectoryMetaInput.h>


class TrajectoryTrackingController {
public:
    TrajectoryTrackingController();
    ~TrajectoryTrackingController();
private:
    void metaInputCB();
    void driveStateCB();
    void processMetaInput();
    // this is where the control magic happens
    void trajectoryCB(const drive_ros_msgs::DrivingLineConstPtr &msg);

    // store current driving state
    float cur_v=0;
    float cur_angle_f=0;
    float cur_angle_r=0;
};


#endif //DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H
