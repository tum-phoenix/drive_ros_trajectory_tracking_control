//
// Created by sebastian on 11.07.19.
//

#include "../include/drive_ros_trajectory_tracking_control/flatness_controller.h"
flatness_controller::flatness_controller(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh)
{

    pnh_.getParam("penalty_y", weight_y_);
    pnh_.getParam("penalty_phi", weight_phi_);
    pnh_.getParam("penalty_front_angle", weight_steeringFront_);
    pnh_.getParam("penalty_rear_angle", weight_steeringRear_);
    //von config einlesen, um live einzustellen

    mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC


    trajectory_sub_ = nh.subscribe("driving_line_topic", 1, &ModelPredictiveController_dlib::trajectoryCB, this);
}

flatness_controller::~flatness_controller() {}

void flatness_controller::trajectoryCB(const drive_ros_msgs::DrivingLineConstPtr &msg) {


    for(int i = 0; i < CHAIN_NUM_NODES; i++){
        nodes_x[i] = msg->points[i].pose.x;
        nodes_y[i] = msg->points[i].pose.y;
    }
    for(int i = 0; i < CHAIN_NUM_NODES-1; i++){
        /*
                const street_environment::TrajectoryPoint &t = (*trajectory)[i];
                const street_environment::TrajectoryPoint &t2 = (*trajectory)[i+1];
                nodes_vMax[i] = std::max<double>(t.velocity,t2.velocity);
                nodes_vMin[i] = std::min<double>(t.velocity,t2.velocity);
                */
        // for first tests:
        nodes_vMax[i] = nodes_v_max_;
        nodes_vMin[i] = nodes_v_min_;
    }







}