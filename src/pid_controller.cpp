

#include "../include/drive_ros_trajectory_tracking_control/pid_controller.h"

PIDController::PIDController(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh) {

//    pnh_.getParam("penalty_y", weight_y_);
//    pnh_.getParam("penalty_phi", weight_phi_);
//    pnh_.getParam("penalty_front_angle", weight_steeringFront_);
//    pnh_.getParam("penalty_rear_angle", weight_steeringRear_);
    //von config einlesen, um live einzustellen

//    mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC
    trajectory_sub_ = nh.subscribe("local_trajectory", 1, &PIDController::trajectoryCB, this);
    esum=0;
    eold=0;
}


PIDController::~PIDController() {}

void PIDController::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {

    auto pid_x = msg->points[0].pose.x;
    auto pid_y = msg->points[0].pose.y;
    auto pid_vx = msg->points[0].twist.x;
    auto pid_vy = msg->points[0].twist.y;


    // get distance x
    float forwardDistanceX = pid_x;
    // get distance y
    float forwardDistanceY = pid_y;

    // handle lane changes and hard-coded turn/drive commands
    float laneChangeDistance = 0.f;
    bool steerFrontAndRear = false;

    drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com =
            drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;

    //calculate steering angle
        //control error
        float e=pid_y;
        esum+=e;
    float kappa;
        kappa = k_p*e+k_d*(e-eold)+k_i*esum;

    //update control errors
    eold=e;

    //saturation
    if (kappa>(22.f*M_PI/180.f)){
        kappa=22.f*M_PI/180.f;
    }
    else if(kappa<(-22.f*M_PI/180.f)){
        kappa=-22.f*M_PI/180.f;
    }

    //print goal point and steering angle
    ROS_INFO_NAMED(stream_name_, "Goal point (%.2f, %.2f)", forwardDistanceX, forwardDistanceY);
    ROS_INFO_NAMED(stream_name_, "Kappa = %.5f", kappa);

    // TODO: laengsbeschleunigung

    float vGoal = pid_vx;

    ROS_INFO_NAMED(stream_name_, "vGoal = %f", vGoal);


    drive_ros_uavcan::phoenix_msgs__NucDriveCommand driveCmdMsg;
    driveCmdMsg.phi_f = kappa;
    if (!steerFrontAndRear)
        driveCmdMsg.phi_r = 0.0f;
    else
        driveCmdMsg.phi_r = kappa;
    driveCmdMsg.lin_vel = vGoal;
    driveCmdMsg.blink_com = blink_com;

    //if(!isnanf(steeringAngleFront) && !isnanf(steeringAngleRear)) {
    ROS_INFO_NAMED(stream_name_, "Steering front = %.1f[deg]", driveCmdMsg.phi_f * 180.f / M_PI);
    ROS_INFO_NAMED(stream_name_, "Steering rear  = %.1f[deg]", driveCmdMsg.phi_r * 180.f / M_PI);

    nuc_command_pub_.publish(driveCmdMsg);
    ROS_INFO_NAMED(stream_name_, "Published uavcan message");

}
