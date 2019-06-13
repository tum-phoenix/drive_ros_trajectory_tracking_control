#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>
//#include <drive_ros_uavcan/phoenix_msgs__DriveState.h>

TrajectoryTrackingController::TrajectoryTrackingController(){}
TrajectoryTrackingController::~TrajectoryTrackingController(){}

void TrajectoryTrackingController::metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
    vMax = msg->max_speed;
    drivingCommand = msg->control_metadata;
}

void TrajectoryTrackingController::driveStateCB(const drive_ros_uavcan:: &msg){
    cur_v=msg.v;
    cur_angle_f=msg.steer_f;
    cur_angle_r=msg.steer_r;
}

void TrajectoryTrackingController::processMetaInput() {
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;
    switch (drivingCommand) {
        case (drive_ros_msgs::TrajectoryMetaInput::STANDARD):
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT):
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT):
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT):
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT):
            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
            break;
        case (drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD):
            break;
    }

    // todo: publish blink command
}

virtual void TrajectoryTrackingController::trajectoryCB(){

}
