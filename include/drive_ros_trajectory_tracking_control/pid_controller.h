
#ifndef SRC_PID_CONTROLLER_H
#define SRC_PID_CONTROLLER_H
#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>


class PIDController : public TrajectoryTrackingController{
public:
    PIDController(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~PIDController();
private:
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);

    std::string stream_name_ = "PIDController";

    // control parameters

    // sum of control errors
    float esum;

    // old control error
    float eold;

    // pid gains
    float k_p=60.0;
    float k_d=50.0;
    float k_i=2.0;

};

#endif //SRC_PID_CONTROLLER
