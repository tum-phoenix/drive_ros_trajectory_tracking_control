//
// Created by sebastian on 02.07.19.
//

#ifndef SRC_MODEL_PREDICTIVE_CONTROLLER_DLIB_H
#define SRC_MODEL_PREDICTIVE_CONTROLLER_DLIB_H
#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>
#include <drive_ros_msgs/TrajectoryPoint.h>
#include <cmath>

#pragma GCC diagnostic ignored "-Wpedantic"
#include <dlib/control.h>         /* no diagnostic for this one */
#pragma GCC diagnostic pop

class ModelPredictiveController_dlib : public TrajectoryTrackingController{
public:
    ModelPredictiveController_dlib(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController_dlib();
private:
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);

    std::string stream_name_ = "ModelPredictiveController_dlib";

    // control parameters
    double weight_y_;
    double weight_phi_;
    double weight_steeringFront_;
    double weight_steeringRear_;
    float minForwardDist_=0.0; //war 1.0 in gernerator
    double l=0.2405;
    static constexpr int horizon_length=4;

};


#endif //SRC_MODEL_PREDICTIVE_CONTROLLER_DLIB_H
