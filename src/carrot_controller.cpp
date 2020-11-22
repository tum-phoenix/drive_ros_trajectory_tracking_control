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
//    for(int i = 0; i < CHAIN_NUM_NODES; i++){
//        nodes_x[i] = msg->points[i].pose.x;
//        nodes_y[i] = msg->points[i].pose.y;
//    }
    auto carrot_x = msg->points[4].pose.x;
    auto carrot_y = msg->points[4].pose.y;
    auto carrot_vx = msg->points[4].twist.x;
    auto carrot_vy = msg->points[4].twist.y;

//    for(int i = 0; i < CHAIN_NUM_NODES-1; i++){
        /*
                const street_environment::TrajectoryPoint &t = (*trajectory)[i];
                const street_environment::TrajectoryPoint &t2 = (*trajectory)[i+1];
                nodes_vMax[i] = std::max<double>(t.velocity,t2.velocity);
                nodes_vMin[i] = std::min<double>(t.velocity,t2.velocity);
                */
        // for first tests:
//        nodes_vMax[i] = nodes_v_max_;
//        nodes_vMin[i] = nodes_v_min_;
//    }
// ===========================
// 			velocity
// ===========================

    // calculate forward velocity
//    float forwardDistanceX = minForwardDist + std::abs(currentVelocity) * k1;
//    forwardDistanceX = hardcodedForwardDistance; //std::min(forwardDistanceX, msg->detectionRange); // limit to detectionRange
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
//    switch (drivingCommand) {
//        case (drive_ros_msgs::TrajectoryMetaInput::STANDARD):
//             nothing to do
//            break;
//        case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT):
//             shift lane distance to the left
//            laneChangeDistance = laneWidth;
//            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
//            steerFrontAndRear = true;
//            break;
//        case (drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT):
//             shift lane distance to the right
//            laneChangeDistance = -laneWidth;
//            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
//            steerFrontAndRear = true;
//            break;
//        case (drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT):
//             hard-code steering angle to the left
//            presetSteeringAngle = crossingTurnAngleLeft;
//            steeringAngleFixed = true;
//            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_LEFT;
//            break;
//        case (drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT):
//             hard code steering angle to the right
//            presetSteeringAngle = -crossingTurnAngleRight;
//            steeringAngleFixed = true;
//            blink_com = drive_ros_uavcan::phoenix_msgs__NucDriveCommand::BLINK_RIGHT;
//            break;
//        case (drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD):
//             fix steering to go straight
//            presetSteeringAngle = 0.f;
//            steeringAngleFixed = true;
//            break;
//    }

//    forwardDistanceY = compute_polynomial_at_location(msg, forwardDistanceX);
    forwardDistanceY = carrot_y;
    // compute derivative on carrot point to get normal if we need to offset ortogonally (lane change)
//    if (laneChangeDistance != 0.f) {
//        float derivative = derive_polynomial_at_location(msg, forwardDistanceX);
//
//         compute normal vector
//        float vec_x = -derivative;
//        float vec_y = 1.f;
//        float normed_distance = std::sqrt(std::pow(derivative, 2) + std::pow(1.f, 2));
//        forwardDistanceX = forwardDistanceX+vec_x*(laneChangeDistance/normed_distance);
//        forwardDistanceY = forwardDistanceY+vec_y*(laneChangeDistance/normed_distance);
//    }

    float kappa;
    if (!steeringAngleFixed)
        kappa = (std::atan2(forwardDistanceY, forwardDistanceX));
    else
        kappa = presetSteeringAngle;

    ROS_INFO_NAMED(stream_name_, "Goal point (%.2f, %.2f)", forwardDistanceX, forwardDistanceY);
    ROS_INFO_NAMED(stream_name_, "Kappa = %.5f", kappa);

    // TODO: querbeschleunigung

    float vGoal = carrot_vx;

    ROS_INFO_NAMED(stream_name_, "vGoal = %f", vGoal);

    // ===========================
    // 		steering angles
    // ===========================

//    float phiAtGoalX = 0.f; // deviate the polynom
//    for(int i = 1; i <= msg->polynom_order; i++) {
//        float tmp = msg->polynom_params.at(i) * i;
//        for(int j = 1; j < i; j++) {
//            tmp *= forwardDistanceX;
//        }
//        phiAtGoalX += tmp;
//    }

//    ROS_INFO_NAMED(stream_name_, "polynom(x)  = %.2f", forwardDistanceY);
//    ROS_INFO_NAMED(stream_name_, "polynom'(x) = %.2f", phiAtGoalX);
//
//     radius is also turnRadiusY
//    float radius = forwardDistanceY / (1.f - std::sin(M_PI_2 - phiAtGoalX));
//
//    float turnRadiusX = -((forwardDistanceY * std::cos(M_PI_2 - phiAtGoalX)) / (1.f - std::sin(M_PI_2 - phiAtGoalX))) + forwardDistanceX;
//
//    ROS_INFO_NAMED(stream_name_, "Turning point (%.2f, %.2f)", turnRadiusX, radius);
//
//    float steeringAngleRear  = - std::atan(turnRadiusX                  / (radius + (0.001f*(radius == 0.f))));
//    float steeringAngleFront = - std::atan((turnRadiusX - axisDistance) / (radius + (0.001f*(radius == 0.f))));

    //ROS_INFO("Steering front = %.1f[deg]", steeringAngleFront * 180.f / M_PI);
    //ROS_INFO("Steering rear  = %.1f[deg]", steeringAngleRear * 180.f / M_PI);

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