//
// Created by seb on 22.11.20.
//

#ifndef SRC_CARROT_CONTROLLER_H
#define SRC_CARROT_CONTROLLER_H
#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>


class CarrotController : public TrajectoryTrackingController{
    public:
        CarrotController(ros::NodeHandle nh, ros::NodeHandle pnh);
        ~CarrotController();
    private:
        void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);

        std::string stream_name_ = "CarrotController";

        // control parameters
        int delay_;

        int max_num_iter_ = 100;


        double alpha_ = 0.5;
        double beta_1_ = 0.7;
        double beta_2_ = 1;
        // angle bounds for front (1) and rear (2)

        double q_diag[NUM_STATES]; //	stage state cost matrix Q diagonal, len = NUM_STATES
        double r_diag[NUM_INPUTS]; //	stage input cost matrix R diagonal, len = NUM_INPUTS
        double p_diag[NUM_STATES];

};

#endif //SRC_CARROT_CONTROLLER_H
