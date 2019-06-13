#ifndef DRIVE_MODEL_PREDICTIVE_CONTROLLER_H
#define DRIVE_MODEL_PREDICTIVE_CONTROLLER_H

#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>

class ModelPredictiveController : public TrajectoryTrackingController{
    ModelPredictiveController(ros::NodeHandle nh);
    ~ModelPredictiveController();
private:
    void trajectoryCB(const drive_ros_msgs::DrivingLineConstPtr &msg);
};


#endif //DRIVE_MODEL_PREDICTIVE_CONTROLLER_H