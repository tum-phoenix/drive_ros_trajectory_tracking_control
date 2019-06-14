#include <drive_ros_trajectory_tracking_control/model_predictive_controller.h>
#include <eigen3/Eigen/Dense>
#include <cppad/cppad.hpp>
#define HAVE_CSTDDEF
#include <cppad/ipopt/solve.hpp>
#undef HAVE_CSTDDEF
#include <cmath>
extern "C"{
#include "drive_ros_trajectory_tracking_control/andromeda.h"
}

ModelPredictiveController::ModelPredictiveController(ros::NodeHandle nh, ros::NodeHandle pnh):
    TrajectoryTrackingController(nh, pnh)
{
}

ModelPredictiveController::~ModelPredictiveController() {}

void ModelPredictiveController::trajectoryCB(const drive_ros_msgs::DrivingLineConstPtr &msg) {
  int delay;
  pnh_.getParam("delay", delay);
  if(delay < 0 || delay >= HORIZON_LEN){
    ROS_ERROR("Invalid stagePrediction");
    return false;
  }

  //get trajectory with distance between points
  double link_length
  nh_.getParam("link_length",link_length);
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
  double max_lateral_acc
      nh.getParam("max_lateral_acc",max_lateral_acc);
  double max_num_iter = 100;
  double alpha = 0.5;
  double beta_1 = 0.7;
  double beta_2 = 1;
  double q_diag[NUM_STATES];//	stage state cost matrix Q diagonal, len = NUM_STATES
  double r_diag[NUM_INPUTS];//	stage input cost matrix R diagonal, len = NUM_INPUTS
  double p_diag[NUM_STATES];
  const double u_1_ub
      nh.getParam("front_angle_rate_Bound",u_1_ub);
  const double u_1_lb = -u_1_ub;
  const double u_2_ub
      nh.getParam("rear_angle_rate_Bound",u_2_ub);
  const double u_2_lb = -u_2_ub;

  //car state
  currentCarState[0] = 0;
  currentCarState[1] = 0;
  currentCarState[2] = cur_angle_f; //car input? aus LMS? -> ros
  currentCarState[3] = cur_angle_r;


  nh.getParam("penalty_y",q_diag[0]);
  nh.getParam("penalty_phi",q_diag[1]);
  nh.getParam("penalty_frontAngle",q_diag[2]);
  nh.getParam("penalty_rearAngle",q_diag[3]);

  //path
  for(int i = 0; i < NUM_STATES; i++){
    p_diag[i] = q_diag[i];
  }
  nh.getParam("penalty_frontAngle_rate",r_diag[0]);
  nh.getParam("penalty_rearAngle_rate",r_diag[1]);


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
    nh.getParam("node_MaxSpeed",nodes_vMax[i]);
    nh.getParam("node_MinSpeed",nodes_vMin[i]);
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
