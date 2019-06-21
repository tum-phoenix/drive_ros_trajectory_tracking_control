#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>

TrajectoryTrackingController::TrajectoryTrackingController(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh)
{
    meta_input_sub_ = nh_.subscribe("meta_in", 10, &TrajectoryTrackingController::metaInputCB, this);
    drive_state_sub_ = nh_.subscribe("uavcan_topic", 5, &TrajectoryTrackingController::driveStateCB, this);
    nuc_command_pub_ = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("drive_command_out", 5);

    pnh_.getParam("link_length",link_length_);
    pnh_.getParam("max_lateral_acc", max_lateral_acc_);

    pnh_.getParam("/front_angle_rate_Bound", u_1_ub_);
    u_1_lb_ = -u_1_ub_;
    pnh_.getParam("/rear_angle_rate_Bound", u_2_ub_);
    u_2_lb_ = -u_2_ub_;

    pnh_.getParam("/node_MaxSpeed", nodes_v_max_);
    pnh_.getParam("/node_MinSpeed", nodes_v_min_);
    ROS_INFO_STREAM("Init Completed");
}

TrajectoryTrackingController::~TrajectoryTrackingController(){}

void TrajectoryTrackingController::metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
    v_max_ = msg->max_speed;
    driving_command_ = msg->control_metadata;
}

void TrajectoryTrackingController::driveStateCB(const drive_ros_uavcan::phoenix_msgs__DriveStateConstPtr &msg){
    cur_v_ = msg->v;
    cur_angle_f_ = msg->steer_f;
    cur_angle_r_ = msg->steer_r;
    ROS_INFO_STREAM("Set current angle");
}

void TrajectoryTrackingController::processMetaInput() {
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand cmd_msg;

    drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;
    switch (driving_command_) {
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

    nuc_command_pub_.publish(cmd_msg);
}

void TrajectoryTrackingController::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg)
{
    ROS_INFO("TrajectoryTrackingController::trajectoryCB called, this should not happen!");
}

