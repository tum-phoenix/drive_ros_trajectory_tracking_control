#ifndef DRIVE_MODEL_PREDICTIVE_CONTROLLER_H
#define DRIVE_MODEL_PREDICTIVE_CONTROLLER_H

#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>

extern "C"{
#include "drive_ros_trajectory_tracking_control/andromeda.h"
}

class ModelPredictiveController : public TrajectoryTrackingController{
public:
    ModelPredictiveController(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController();
private:
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);

    std::string stream_name_ = "ModelPredictiveController";

    // control parameters
    int delay_;

    int max_num_iter_ = 100;

    double link_length_;
    double max_lateral_acc_;
    double alpha_ = 0.5;
    double beta_1_ = 0.7;
    double beta_2_ = 1;
    // angle bounds for front (1) and rear (2)
    double u_1_ub_;
    double u_1_lb_;
    double u_2_ub_;
    double u_2_lb_;

    double q_diag[NUM_STATES]; //	stage state cost matrix Q diagonal, len = NUM_STATES
    double r_diag[NUM_INPUTS]; //	stage input cost matrix R diagonal, len = NUM_INPUTS
    double p_diag[NUM_STATES];

    double nodes_v_max_;
    double nodes_v_min_;
};


#endif //DRIVE_MODEL_PREDICTIVE_CONTROLLER_H
