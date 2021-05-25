#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/model_predictive_controller.h>
#include <drive_ros_trajectory_tracking_control/carrot_controller.h>
#include <drive_ros_trajectory_tracking_control/pid_controller.h>
#include <drive_ros_trajectory_tracking_control/model_predictive_controller_dlib.h>
#include <drive_ros_trajectory_tracking_control/model_predictive_controller_nonlin.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_tracking_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    double cycle_t;

    std::string controller_type{"PID"};
    pnh.param<std::string>("controller_type", controller_type, "PID");

    std::unique_ptr<TrajectoryTrackingController> controller;

    if (controller_type == "MPC") {
        controller.reset(new ModelPredictiveController(nh, pnh));
    } else if (controller_type == "carrot") {
        controller.reset(new CarrotController(nh, pnh));
    } else if (controller_type == "PID") {
        controller.reset(new PIDController(nh, pnh));
    } else if (controller_type == "MPC_DLIB") {
        controller.reset(new ModelPredictiveController_dlib(nh, pnh));
    } else if (controller_type == "MPC_NONLIN") {
        controller.reset(new ModelPredictiveController_nonlin(nh, pnh));
    } else {
        ROS_ERROR_STREAM("Invalid controller type provided: " << controller_type << ", shutting down");
        return 0;
    }

    nh.getParam("cycletime", cycle_t);
    ros::Rate loop_rate(1/cycle_t);
    while (ros::ok()) {
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}
