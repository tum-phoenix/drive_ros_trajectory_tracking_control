//
// Created by sebastian on 19.05.19.
//

#include "../include/drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h"



trajectory_tracking_controller::trajectory_tracking_controller(){}
trajectory_tracking_controller::~trajectory_tracking_controller(){}

void trajectory_tracking_controller::blink_check(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
    vMax = msg->max_speed;
    drivingCommand = msg->control_metadata;
}

void trajectory_tracking_controller::set_current_egomotion(const drieve_teensy_main::5009.DriveState.uavcan &msg){
cur_v=msg.v;
cur_angle_f=msg.steer_f;
cur_angle_r=msg.steer_r;
}

drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type trajectory_tracking_controller::blink(){
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;
    switch (drivingCommand) {
        case (drive_ros_msgs::TrajectoryMetaInput::STANDARD):
            // nothing to do
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT):
            // shift lane distance to the left
            laneChangeDistance = laneWidth;
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
            steerFrontAndRear = true;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT):
            // shift lane distance to the right
            laneChangeDistance = -laneWidth;
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
            steerFrontAndRear = true;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT):
            // hard-code steering angle to the left
            presetSteeringAngle = crossingTurnAngleLeft;
            steeringAngleFixed = true;
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT):
            // hard code steering angle to the right
            presetSteeringAngle = -crossingTurnAngleRight;
            steeringAngleFixed = true;
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD):
            // fix steering to go straight
            presetSteeringAngle = 0.f;
            steeringAngleFixed = true;
            break;
    }
    return blink_com
}

virtual void trajectory_tracking_controller::control_values(){

}