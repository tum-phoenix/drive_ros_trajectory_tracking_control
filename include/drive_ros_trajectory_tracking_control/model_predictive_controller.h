#ifndef DRIVE_MODEL_PREDICTIVE_CONTROLLER_H
#define DRIVE_MODEL_PREDICTIVE_CONTROLLER_H

#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>

extern "C"{
#include "drive_ros_trajectory_tracking_control/andromeda.h"
}

#include <cmath>

class ModelPredictiveController : public TrajectoryTrackingController{
public:
    ModelPredictiveController(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController();
private:
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);
    drive_ros_msgs::Trajectory getTrajectoryWithDistance(double distance, const drive_ros_msgs::TrajectoryConstPtr &trajectory);

    std::string stream_name_ = "ModelPredictiveController";

    // control parameters
    int delay_=0;

    int max_num_iter_ = 100;


    double q_diag[NUM_STATES]; //	stage state cost matrix Q diagonal, len = NUM_STATES
    double r_diag[NUM_INPUTS]; //	stage input cost matrix R diagonal, len = NUM_INPUTS
    double p_diag[NUM_STATES];
    double alpha_ = 0.5;
    double beta_1_ = 0.7;
    double beta_2_ = 1;

    double cur_angle_f_=0;
    double cur_angle_r_=0;

};


#endif //DRIVE_MODEL_PREDICTIVE_CONTROLLER_H
