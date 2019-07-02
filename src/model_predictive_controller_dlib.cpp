//
// Created by sebastian on 02.07.19.
//

#include "../include/drive_ros_trajectory_tracking_control/model_predictive_controller_dlib.h"

ModelPredictiveController_dlib::ModelPredictiveController_dlib(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh)
{

    pnh_.getParam("penalty_y", weight_y);
    pnh_.getParam("penalty_phi", weight_phi);
    pnh_.getParam("penalty_front_angle", weight_steeringFront);
    pnh_.getParam("penalty_rear_angle", weight_steeringRear);
    //von config einlesen, um live einzustellen

    mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC


    trajectory_sub_ = nh.subscribe("trajectory_in", 1, &ModelPredictiveController::trajectoryCB, this);
}

ModelPredictiveController_dlib::~ModelPredictiveController() {}

void ModelPredictiveController_dlib::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {
    //Carrot... da diskretisiert schwierig....
    double v =cur_v_
    if(fabs(v) < 0.1){
        logger.debug("cycle")<<"car is slow: "<<car->velocity();
        v=0.1;//Some controller has some issue divides by v without error-checking
    }
    float forwardDistanceX = minForwardDist + std::abs(v)
    float distanceToTrajectoryPoint =  compute_polynomial_at_location(msg, forwardDistanceX);
    //const float distanceSearched = config().get<float>("distanceRegelpunkt", 0.50);

    //get the trajectory point
    street_environment::TrajectoryPoint trajectoryPoint = getTrajectoryPoint(distanceToTrajectoryPoint);

    double phi_soll = atan2(trajectoryPoint.directory.y, trajectoryPoint.directory.x);
    double y_soll = trajectoryPoint.position.y;

    logger.debug("phi y v") << phi_soll << " "<< y_soll<<" "<< v;

    double steering_front, steering_rear;

    const int STATES = 2; //number of states (y and phi)
    const int CONTROLS = 2; //number of control inputs (steering_front and steering_rear)
    double T = mpcParameters.stepSize;

    // Modell festlegen
    // x_{i+1} == A*x_i + B*u_i + C

    //HACK:
    //anstatt v konstant zu lassen folgende Idee:
    //Wenn v größer wird, kann im betrachteten Prädiktionshorizont das Regelziel schneller erreicht werden.
    //Das führt dann (meiner Meinung nach) tendenziell zu kleineren Lenkwinkeln. Wir wollen aber eigentlich
    //genau das Gegenteil, nämlich größere Lenkwinkel bei höherer Geschwindigkeit, um Querschlupf entgegenzuwirken.
    //Lösungsansatz:
    //Vorgabe einer Grundgeschwindigkeit v0 (z.B. v0=1), auf die der Regler bei langsamer Fahrt eingestellt wird.
    //Dann abhängig von der wirklichen aktuellen Geschwindigkeit Addition eines "Geschwindigkeitsfaktors"
    // ==> v_regler = 1 + c*v_real oder alternativ v_regler = exp(-c*v_real)

    //if (config().get("velocityFactor", 0.0)) v = std::exp(-v*config().get("velocityFactor", 0.0));
    //v = std::max(1.0, v);

    dlib::matrix<double,STATES,STATES> A;
    A = 1, T*v, 0, 1;

    dlib::matrix<double,STATES,CONTROLS> B;
    B = 0, T*v, T*v/l, -T*v/l;

    dlib::matrix<double,STATES,1> C; //keine konstante Stoerung
    C = 0, 0;

    dlib::matrix<double,STATES,1> Q; //Gewichtung der Regelabweichung fuer Querabstand y und Orientierung phi
    Q = mpcParameters.weight_y, mpcParameters.weight_phi;

    dlib::matrix<double,STATES,1> R; //Gewichtung der Stellgroessen
    R = mpcParameters.weight_steeringFront, mpcParameters.weight_steeringRear;


    dlib::mpc<STATES,CONTROLS,MPC_HORIZON> controller(A,B,C,Q,R,lower,upper); //30*T ist der Zeithorizont fuer die praediktion, d.h. 30 Zeitschritte wird in die Zukunft simuliert

    dlib::matrix<double,STATES,1> target;
    target = delta_y, delta_phi;

    controller.set_target(target);

    //Stellschrauben fuer performance
    //controller.set_epsilon(0.05);
    //controller.set_max_iterations(300);

    dlib::matrix<double,STATES,1> current_state;
    current_state = 0, 0;

    dlib::matrix<double,CONTROLS,1> action = controller(current_state); //loese MPC Problem


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
    //set trajectoryPoint for debugging;
    *debugging_trajectoryPoint = trajectoryPoint;

    if(trajectoryPoint.velocity == 0){
        state.state = street_environment::CarCommand::StateType::IDLE;
    }else{
        state.state = street_environment::CarCommand::StateType::DRIVING;
    }
    return;
}