#include "../include/drive_ros_trajectory_tracking_control/carrot_controller.h"

CarrotController::CarrotController(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh) {

//    pnh_.getParam("penalty_y", weight_y_);
//    pnh_.getParam("penalty_phi", weight_phi_);
//    pnh_.getParam("penalty_front_angle", weight_steeringFront_);
//    pnh_.getParam("penalty_rear_angle", weight_steeringRear_);
    //von config einlesen, um live einzustellen

//    mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC
    trajectory_sub_ = nh.subscribe("/trajectory_publisher", 1, &CarrotController::trajectoryCB, this);
}


CarrotController::~CarrotController() {}

void CarrotController::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {

    auto carrot_x = msg->points[4].pose.x;
    auto carrot_y = msg->points[4].pose.y;
    auto carrot_vx = msg->points[4].twist.x;
    auto carrot_vy = msg->points[4].twist.y;

    float forwardDistanceX = carrot_x;
    // get y from polynom
    float forwardDistanceY = 0.f;

    // handle lane changes and hard-coded turn/drive commands
    float laneChangeDistance = 0.f;
    float presetSteeringAngle;
    bool steeringAngleFixed = false;
    bool steerFrontAndRear = false;

    drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com =
            drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;

    forwardDistanceY = carrot_y;

    float kappa;
    if (!steeringAngleFixed)
        kappa = (std::atan2(forwardDistanceY, forwardDistanceX));
    else
        kappa = presetSteeringAngle;

    ROS_INFO_NAMED(stream_name_, "Goal point (%.2f, %.2f)", forwardDistanceX, forwardDistanceY);
    ROS_INFO_NAMED(stream_name_, "Kappa: = %.5f", kappa);

    // TODO: querbeschleunigung

    float vGoal = carrot_vx;

    ROS_INFO_NAMED(stream_name_, "vGoal = %f", vGoal);

    drive_ros_uavcan::phoenix_msgs__NucDriveCommand driveCmdMsg;
    driveCmdMsg.phi_f = -kappa;
    if (!steerFrontAndRear)
        driveCmdMsg.phi_r = 0.0f;
    else
        driveCmdMsg.phi_r = -kappa;
    driveCmdMsg.lin_vel = carrot_vx;
    driveCmdMsg.blink_com = blink_com;

    //if(!isnanf(steeringAngleFront) && !isnanf(steeringAngleRear)) {
    ROS_INFO_NAMED(stream_name_, "Steering front = %.1f[deg]", driveCmdMsg.phi_f * 180.f / M_PI);
    ROS_INFO_NAMED(stream_name_, "Steering rear  = %.1f[deg]", driveCmdMsg.phi_r * 180.f / M_PI);

    nuc_command_pub_.publish(driveCmdMsg);
    ROS_INFO_NAMED(stream_name_, "Published uavcan message");

}