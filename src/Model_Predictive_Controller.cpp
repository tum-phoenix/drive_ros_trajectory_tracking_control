//
// Created by sebastian on 19.05.19.
//

#include "../include/drive_ros_trajectory_tracking_control/Model_Predictive_Controller.h"
#include <eigen3/Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cmath>
extern "C"{
#include "andromeda.h"
}

Model_Predictive_Controller::Model_Predictive_Controller(ros::NodeHandle nh) {
    trajectory_meta_sub_ = nh_.subscribe("meta_in", 10, &Model_Predictive_Controller::blink_check, this);
    ego_motion = nh.subscribe("")
    trajectory=nh.subscribe("",1,Model_Predictive_Controller::control_values)//street_environment::Trajectory??
}

Model_Predictive_Controller::~Model_Predictive_Controller() {}

void Model_Predictive_Controller::blink_check(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg) {
    trajectory_tracking_controller::blink_check(msg);
    //vMax = msg->max_speed;
    //drivingCommand = msg->control_metadata;
}

void Model_Predictive_Controller::set_current_egomotion(const drieve_teensy_main::5009.DriveState.uavcan &msg) {
    trajectory_tracking_controller::(msg);
    //cur_v=msg.v;
    //cur_angle_f=msg.steer_f;
    //cur_angle_r=msg.steer_r;
}


void Model_Predictive_Controller::control_values(const drive_ros_msgs::DrivingLineConstPtr &msg){
        int delay =  config().get<int>("stagePrediction",0); //wo bekommen wir die kacke her? Step length
        if(delay < 0 || delay >= HORIZON_LEN){
            logger.error("invalid stagePrediction")<<delay;
            return false;
        }

        //get trajectory with distance between points
        double link_length = config().get<double>("link_length",0.1);
        street_environment::Trajectory tr = trajectory->getWithDistanceBetweenPoints(link_length); // Ros!
        if(tr.size() < CHAIN_NUM_NODES){
            logger.error("INVALID path given")<< tr.size();
            trajectoryDebug->clear();
            return false;
        }
        tr.erase(tr.begin()+CHAIN_NUM_NODES,tr.end());
        *trajectoryDebug = tr;

        //Inputs
        double currentCarState[NUM_STATES];
        double nodes_x[CHAIN_NUM_NODES];
        double nodes_y[CHAIN_NUM_NODES];
        double nodes_vMin[CHAIN_NUM_NODES-1];
        double nodes_vMax[CHAIN_NUM_NODES-1];
        double max_lateral_acc = config().get<double>("max_lateral_acc",1);
        double max_num_iter = 100;
        double alpha = 0.5;
        double beta_1 = 0.7;
        double beta_2 = 1;
        double q_diag[NUM_STATES];//	stage state cost matrix Q diagonal, len = NUM_STATES
        double r_diag[NUM_INPUTS];//	stage input cost matrix R diagonal, len = NUM_INPUTS
        double p_diag[NUM_STATES];
        const double u_1_ub = config().get<double>("front_angle_rate_Bound",1);
        const double u_1_lb = -u_1_ub;
        const double u_2_ub = config().get<double>("rear_angle_rate_Bound",1);;
        const double u_2_lb = -u_2_ub;

        //car state
        currentCarState[0] = 0;
        currentCarState[1] = 0;
        currentCarState[2] = cur_angle_f; //car input? aus LMS? -> ros
        currentCarState[3] = cur_angle_r;

        q_diag[0] = config().get<double>("penalty_y",10);   //config -> ros parameter?
        q_diag[1] = config().get<double>("penalty_phi",10);
        q_diag[2] = config().get<double>("penalty_frontAngle",1);
        q_diag[3] = config().get<double>("penalty_rearAngle",1);
        //path
        for(int i = 0; i < NUM_STATES; i++){
            p_diag[i] = q_diag[i];
        }
        r_diag[0] = config().get<double>("penalty_frontAngle_rate",100);    //ros parameter
        r_diag[1] = config().get<double>("penalty_rearAngle_rate",100);


        //set input date
        for(int i = 0; i < CHAIN_NUM_NODES; i++){
            const street_environment::TrajectoryPoint &t = tr[i];   //enviornment to ros
            nodes_x[i] = t.position.x;
            nodes_y[i] = t.position.y;
        }
        for(int i = 0; i < CHAIN_NUM_NODES-1; i++){
            /*
            const street_environment::TrajectoryPoint &t = (*trajectory)[i];
            const street_environment::TrajectoryPoint &t2 = (*trajectory)[i+1];
            nodes_vMax[i] = std::max<double>(t.velocity,t2.velocity);
            nodes_vMin[i] = std::min<double>(t.velocity,t2.velocity);
            */
            //for first tests:
            nodes_vMax[i] = config().get<double>("node_MaxSpeed",0.5);  //rosconfig wozu braucht man das?
            nodes_vMin[i] = config().get<double>("node_MinSpeed",0.5);  //unstraints?
        }


        //outputs
        double v_star[HORIZON_LEN];
        double u_1_star[HORIZON_LEN];   //Horizon_len ? Präditionshorrizont in steps?
        double u_2_star[HORIZON_LEN];

        call_andromeda(currentCarState,q_diag,r_diag,p_diag,nodes_x,nodes_y,link_length,nodes_vMin,nodes_vMax,
                       max_lateral_acc,max_num_iter,alpha,beta_1,beta_2,u_1_lb,u_1_ub,u_2_lb,u_2_ub,v_star,u_1_star,u_2_star);

        //Set values
        state.steering_front = car->steeringFront() + u_1_star[delay]; //publish as driving command
        state.steering_rear = car->steeringRear()+u_2_star[delay];
        state.targetSpeed = v_star[delay];

        drive_ros_uavcan::phoenix_msgs__NucDriveCommand driveCmdMsg;
        driveCmdMsg.phi_f = -kappa*understeerFactor;
        driveCmdMsg.phi_r = -kappa*understeerFactor;

        ROS_INFO_NAMED(stream_name_, "Steering front = %.1f[deg]", driveCmdMsg.phi_f * 180.f / M_PI);
        ROS_INFO_NAMED(stream_name_, "Steering rear  = %.1f[deg]", driveCmdMsg.phi_r * 180.f / M_PI);

        state.targetDistance = 1; //TODO dont think that we even need it
        logger.timeEnd("mikMPC");

}

drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type Model_Predictive_Controller::blink(){

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
