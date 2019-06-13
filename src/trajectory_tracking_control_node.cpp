#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/model_predictive_controller.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_tracking_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ModelPredictiveController controller(nh);

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}
