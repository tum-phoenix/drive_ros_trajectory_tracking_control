#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/model_predictive_controller.h>
#include <drive_ros_trajectory_tracking_control/carrot_controller.h>
#include <drive_ros_trajectory_tracking_control/pid_controller.h>
#include <drive_ros_trajectory_tracking_control/model_predictive_controller_dlib.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_tracking_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    double cycle_t;
//    ModelPredictiveController controller(nh, pnh);
//    CarrotController controller(nh, pnh);
    PIDController controller(nh, pnh);
//    ModelPredictiveController_dlib controller(nh, pnh);
    pnh.getParam("cycletime", cycle_t);
    ros::Rate loop_rate(1/cycle_t);
    while (ros::ok()) {
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}
