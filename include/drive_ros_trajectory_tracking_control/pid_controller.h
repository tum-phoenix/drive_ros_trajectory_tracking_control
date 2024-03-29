
#ifndef SRC_PID_CONTROLLER_H
#define SRC_PID_CONTROLLER_H
#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>
#include <drive_ros_msgs/TrajectoryPoint.h>


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
    float k_p;
    float k_d;
    float k_i;

    //params for velocity PID
    double k_p_vel, k_i_vel, k_d_vel;
    double e_old_vel=0.0;
    double e_sum_vel=0.0;

};

#endif //SRC_PID_CONTROLLER
