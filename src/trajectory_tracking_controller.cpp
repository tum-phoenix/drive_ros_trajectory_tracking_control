#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>

TrajectoryTrackingController::TrajectoryTrackingController(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh)
{
    meta_input_sub_ = nh_.subscribe("meta_in", 10, &TrajectoryTrackingController::metaInputCB, this);
    drive_state_sub_ = nh_.subscribe("drive_state_in", 5, &TrajectoryTrackingController::driveStateCB, this);
    nuc_command_pub_ = nh_.advertise<drive_ros_uavcan::phoenix_msgs__NucDriveCommand>("drive_command_out", 5);

    pnh_.getParam("link_length",link_length_);
    pnh_.getParam("max_lateral_acc", max_lateral_acc_);

    pnh_.getParam("front_angle_rate_Bound", u_1_ub_);
    u_1_lb_ = -u_1_ub_;
    pnh_.getParam("rear_angle_rate_Bound", u_2_ub_);
    u_2_lb_ = -u_2_ub_;

    pnh_.getParam("node_max_speed", nodes_v_max_);
    pnh_.getParam("node_min_speed", nodes_v_min_);
    nh_.getParam("cycletime", cycle_t_);
    pnh_.getParam("angle_bound",angle_bound);

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

drive_ros_msgs::TrajectoryPoint TrajectoryTrackingController::getTrajectoryPoint(
        const double distanceToPoint , const drive_ros_msgs::TrajectoryConstPtr &trajectory){
    //if we find nothing, we just want to idle forward
    drive_ros_msgs::TrajectoryPoint trajectoryPoint;
    //x-Pos
    trajectoryPoint.pose.x = distanceToPoint;
    //y-Pos
    trajectoryPoint.pose.y = 0;
    //x-velocity
    trajectoryPoint.twist.x = 0;
    //y-velocity
    trajectoryPoint.twist.y = 0;
    if(trajectory->points.size() == 0){
        ROS_ERROR_STREAM("Can't follow anything");
        return trajectoryPoint;
    }
    bool found = false;

    //old code
    //Nur den Abstand in x-richtung zu nehmen ist nicht schlau, denn wenn das Auto eskaliert eskaliert der Regler noch viel mehr!
    float currentDistance = 0;
    for(int i = 1; i < trajectory->points.size(); i++){
        drive_ros_msgs::TrajectoryPoint bot = trajectory->points[i-1];
        drive_ros_msgs::TrajectoryPoint top = trajectory->points[i];
        float length=sqrt(pow(top.pose.x-bot.pose.x,2)+pow(top.pose.y-bot.pose.y,2));
        currentDistance += length;
        if(currentDistance > distanceToPoint){
            //We start at the bottom-point
            //inerpolate between bot and top! #IMPORTANT (velocity!,viewdir)
            float delta = currentDistance-distanceToPoint;
            float along[2];
            along[0] = ((bot.pose.x-top.pose.x)/length)*delta;
            along[1] = ((bot.pose.y-top.pose.y)/length)*delta;
            trajectoryPoint =  top;
            trajectoryPoint.pose.x = trajectoryPoint.pose.x+along[0];
            trajectoryPoint.pose.y = trajectoryPoint.pose.y+along[1];
            found = true;
            break;
        }
    }

    if(!found){
        ROS_ERROR_STREAM("No trajectoryPoint found, returning the last point of the trajectory:  "<< trajectory->points.size() << " distanceSearched: "<< currentDistance << " distanceToTrajectoryPoint: "<< distanceToPoint);
        trajectoryPoint = trajectory->points[trajectory->points.size()-1];
    }

    //we just return the last Point
    return trajectoryPoint;
}