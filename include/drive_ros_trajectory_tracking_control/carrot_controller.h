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


};

#endif //SRC_CARROT_CONTROLLER_H
