#include <drive_ros_trajectory_tracking_control/model_predictive_controller.h>
#include <eigen3/Eigen/Dense>
#include <cppad/cppad.hpp>
#define HAVE_CSTDDEF
#include <cppad/ipopt/solve.hpp>
#undef HAVE_CSTDDEF
#include <cmath>

ModelPredictiveController::ModelPredictiveController(ros::NodeHandle nh, ros::NodeHandle pnh):
    TrajectoryTrackingController(nh, pnh)
{
  pnh_.getParam("delay", delay_);

  if(delay_ < 0 || delay_ >= HORIZON_LEN){
    ROS_ERROR("Invalid stagePrediction");
    return;
  }

  if (!pnh_.getParam("penalty_y", q_diag[0]));
  pnh_.getParam("penalty_phi", q_diag[1]);
  pnh_.getParam("penalty_front_angle", q_diag[2]);
  pnh_.getParam("penalty_rear_angle", q_diag[3]);
  pnh_.getParam("penalty_front_angle_rate", r_diag[0]);
  pnh_.getParam("penalty_rear_angle_rate", r_diag[1]);

  // path
  for(int i = 0; i < NUM_STATES; i++) {
    p_diag[i] = q_diag[i];
  }

  trajectory_sub_ = nh.subscribe("local_trajectory", 1, &ModelPredictiveController::trajectoryCB, this);
}

ModelPredictiveController::~ModelPredictiveController() {}

void ModelPredictiveController::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {

    //get trajectory with distance between points
drive_ros_msgs::Trajectory trajectory = getTrajectoryWithDistance(link_length_, msg);

    if(trajectory.points.size() < CHAIN_NUM_NODES){
        ROS_ERROR_STREAM("Path with invalid number of nodes given: "<< trajectory.points.size() << " minimum required is: " <<
                     CHAIN_NUM_NODES);
        drive_ros_msgs::TrajectoryPoint point;
        for(int i=trajectory.points.size(); i<CHAIN_NUM_NODES; i++){
            point=trajectory.points[i-1];
            trajectory.points.push_back(point);
        }
    }

  //Inputs
  double currentCarState[NUM_STATES];
  double nodes_x[CHAIN_NUM_NODES];
  double nodes_y[CHAIN_NUM_NODES];
  double nodes_vMin[CHAIN_NUM_NODES-1];
  double nodes_vMax[CHAIN_NUM_NODES-1];


  //car state
  currentCarState[0] = 0;
  currentCarState[1] = 0;
  currentCarState[2] = cur_angle_f_; //car input? aus LMS? -> ros
  currentCarState[3] = cur_angle_r_;

  //set input data
  for(int i = 0; i < CHAIN_NUM_NODES; i++){
    nodes_x[i] = trajectory.points[i].pose.x;
    nodes_y[i] = trajectory.points[i].pose.y;
  }
  for(int i = 0; i < CHAIN_NUM_NODES-1; i++){
    /*
            const street_environment::TrajectoryPoint &t = (*trajectory)[i];
            const street_environment::TrajectoryPoint &t2 = (*trajectory)[i+1];
            nodes_vMax[i] = std::max<double>(t.velocity,t2.velocity);
            nodes_vMin[i] = std::min<double>(t.velocity,t2.velocity);
            */
    // for first tests:
    nodes_vMax[i] = nodes_v_max_;
    nodes_vMin[i] = nodes_v_min_;
  }

  //outputs
  double v_star[HORIZON_LEN];
  double u_1_star[HORIZON_LEN];   //Horizon_len ? Präditionshorrizont in steps?
  double u_2_star[HORIZON_LEN];
  ROS_INFO_STREAM("Nodes 0: " << nodes_x[0] << ", "<< nodes_y[0] << " 1: " << nodes_x[1] << ", "
                                            << nodes_y[1] << " 2: " << nodes_x[2] << ", "<< nodes_y[2]);
  //ROS_INFO_STREAM("Nodes = " << nodes_x[0] << nodes_y[0] << nodes_x[1] << nodes_y[1] << nodes_x[2] << nodes_y[2]);
  call_andromeda(currentCarState,
                 q_diag,
                 r_diag,
                 p_diag,
                 nodes_x,
                 nodes_y,
                 link_length_,
                 nodes_vMin,
                 nodes_vMax,
                 max_lateral_acc_,
                 max_num_iter_,
                 alpha_,
                 beta_1_,
                 beta_2_,
                 u_1_lb_,
                 u_1_ub_,
                 u_2_lb_,
                 u_2_ub_,
                 v_star,
                 u_1_star,
                 u_2_star);

  //Set values

  drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command_msg;

  drive_command_msg.phi_f = cur_angle_f_ + u_1_star[delay_]*cycle_t_; //publish as driving command
  drive_command_msg.phi_r = cur_angle_r_ + u_2_star[delay_]*cycle_t_;

  //maybe insert measured values for angles instead
  cur_angle_f_+=u_1_star[delay_]*cycle_t_;
  cur_angle_r_+=u_2_star[delay_]*cycle_t_;

  //just for real vehicle, in simulation not necessary
  // drive_command_msg.phi_f = -drive_command_msg.phi_f;
  //drive_command_msg.phi_r = -drive_command_msg.phi_r;
  drive_command_msg.lin_vel = v_star[delay_];
  nuc_command_pub_.publish(drive_command_msg);
  ROS_INFO_STREAM( "Steering front = " << drive_command_msg.phi_f * 180.f / M_PI);
  ROS_INFO_STREAM( "Steering rear = " << drive_command_msg.phi_r * 180.f / M_PI);

//  state.targetDistance = 1; //TODO dont think that we even need it
}


drive_ros_msgs::Trajectory ModelPredictiveController::getTrajectoryWithDistance
    (double distance, const drive_ros_msgs::TrajectoryConstPtr &trajectory){
drive_ros_msgs::Trajectory result;
if(distance<0){
    ROS_ERROR_STREAM("invalid distance given");
}
if(trajectory->points.size()==0){
    return result;
}

int currentIndex=1;
result.points.push_back(trajectory->points[0]);
while(true){
    drive_ros_msgs::TrajectoryPoint lastPoint = result.points[result.points.size()-1];
    drive_ros_msgs::TrajectoryPoint next=trajectory->points[currentIndex];

    double currentDistance= sqrt(pow(lastPoint.pose.x-next.pose.x,2)+pow(lastPoint.pose.y-next.pose.y,2));
    while(currentDistance<distance){
        currentIndex++;
        if(currentIndex>=(int)trajectory->points.size()){
            break;
        }
        next=trajectory->points[currentIndex];
        currentDistance= sqrt(pow(lastPoint.pose.x-next.pose.x,2)+pow(lastPoint.pose.y-next.pose.y,2));
    }

    drive_ros_msgs::TrajectoryPoint diff;
    diff.pose.x=next.pose.x-lastPoint.pose.x;
    diff.pose.y=next.pose.y-lastPoint.pose.y;

    //if(currentDistance!=0){
        drive_ros_msgs::TrajectoryPoint newT;
        newT.pose.x=lastPoint.pose.x+(diff.pose.x/currentDistance)*distance;
        newT.pose.y=lastPoint.pose.y+(diff.pose.y/currentDistance)*distance;
        newT.twist=lastPoint.twist;
        result.points.push_back(newT);
    /*}else{
        currentIndex++;
    }*/
    if(currentIndex>=(int)trajectory->points.size()){
        break;
    }
}
return result;
}