//
// Created by sebastian on 02.07.19.
//

#include "../include/drive_ros_trajectory_tracking_control/model_predictive_controller_dlib.h"

ModelPredictiveController_dlib::ModelPredictiveController_dlib(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh)
{

    pnh_.getParam("penalty_y", weight_y_);
    pnh_.getParam("penalty_phi", weight_phi_);
    pnh_.getParam("penalty_front_angle", weight_steeringFront_);
    pnh_.getParam("penalty_rear_angle", weight_steeringRear_);
    //von config einlesen, um live einzustellen

    mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC


    trajectory_sub_ = nh.subscribe("driving_line_topic", 1, &ModelPredictiveController_dlib::trajectoryCB, this);
}

ModelPredictiveController_dlib::~ModelPredictiveController() {}

void ModelPredictiveController_dlib::trajectoryCB(const drive_ros_msgs::DrivingLineConstPtr &msg) {
    //Carrot... da diskretisiert schwierig....
    double v =cur_v_
    if(fabs(v) < 0.1){
        ROS_INFO_STREAM("car is slow: ");
        v=0.1;//Some controller has some issue divides by v without error-checking
    }

    float forwardDistanceX = std::abs(v)* T_
    float forwardDistanceY =  compute_polynomial_at_location(msg, forwardDistanceX);
    //const float distanceSearched = config().get<float>("distanceRegelpunkt", 0.50);

    double phi_soll = atan2(forwardDistanceY, forwardDistanceX);
    double y_soll = forwardDistanceY;

    logger.debug("phi y v") << phi_soll << " "<< y_soll<<" "<< v;

    double steering_front, steering_rear;

    dlib::matrix<double,STATES_,STATES_> A;
    A = 1, T_*v, 0, 1;

    dlib::matrix<double,STATES_,CONTROLS_> B;
    B = 0, T_*v, T_*v/l, -T_*v/l;

    dlib::matrix<double,STATES_,1> C; //keine konstante Stoerung
    C = 0, 0;

    dlib::matrix<double,STATES_,1> Q; //Gewichtung der Regelabweichung fuer Querabstand y und Orientierung phi
    Q = weight_y_, weight_phi_;

    dlib::matrix<double,STATES_,1> R; //Gewichtung der Stellgroessen
    R = weight_steeringFront_, weight_steeringRear_;


    dlib::mpc<STATES_,CONTROLS_,MPC_HORIZON_> controller(A,B,C,Q,R,lower,upper); //30*T ist der Zeithorizont fuer die praediktion, d.h. 30 Zeitschritte wird in die Zukunft simuliert

    dlib::matrix<double,STATES,1> target;
    target = delta_y, delta_phi;

    controller.set_target(target);

    //Stellschrauben fuer performance
    //controller.set_epsilon(0.05);
    //controller.set_max_iterations(300);

    dlib::matrix<double,STATES_,1> current_state;
    current_state = 0, 0;

    dlib::matrix<double,CONTROLS_,1> action = controller(current_state); //loese MPC Problem


    steering_front = action(0,0);
    steering_rear = action(1,0);
    //TODO get angle per time

    logger.debug("trajectory_point_controller") << "lw vorne: " << steering_front*180/M_PI << "  lw hinten: " << steering_rear*180/M_1_PI;
    if(std::isnan(steering_front) || std::isnan(steering_rear || std::isnan(trajectoryPoint.velocity)) ){
        logger.error("trajectory_point_controller: ")<<"invalid vals: " <<steering_front <<" " <<steering_rear ;
    }

    //state.targetDistance = trajectoryPoint.position.length(); //TODO absolutwert no idea?!
    //Set values
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command_msg;

    drive_command_msg.phi_f = steering_front; //publish as driving command
    drive_command_msg.phi_r = steering_rear;
    drive_command_msg.lin_vel = v_max_;
    nuc_command_pub_.publish(drive_command_msg);
    ROS_INFO_STREAM( "Steering front = " << drive_command_msg.phi_f * 180.f / M_PI);
    ROS_INFO_STREAM( "Steering rear = " << drive_command_msg.phi_r * 180.f / M_PI);

    return;
}
