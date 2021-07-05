//
// Created by phillip on 02.05.2021
//

#ifndef SRC_MODEL_PREDICTIVE_CONTROLLER_NONLIN_H
#define SRC_MODEL_PREDICTIVE_CONTROLLER_NONLIN_H
#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>
#include <drive_ros_msgs/TrajectoryPoint.h>
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

// horizon_length and index variables for states and controls
const size_t horizon_length = 4;
const size_t y_idx = 0;
const size_t phi_idx = horizon_length;
const size_t v_idx = phi_idx + horizon_length;
const size_t beta_idx = v_idx + horizon_length;
const size_t yaw_idx = beta_idx + horizon_length;
const size_t a_idx = yaw_idx + horizon_length;
const size_t delta_f_idx = a_idx + horizon_length;
const size_t delta_r_idx = delta_f_idx + horizon_length;
const size_t a_ref_idx = delta_r_idx + horizon_length;
const size_t delta_f_ref_idx = a_ref_idx + horizon_length - 1;
const size_t delta_r_ref_idx = delta_f_ref_idx + horizon_length -1;


class ModelPredictiveController_nonlin : public TrajectoryTrackingController{
public:
    ModelPredictiveController_nonlin(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController_nonlin();
private:
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);
    std::string stream_name_ = "ModelPredictiveController_nonlin";

    // control parameters
    double weight_y_;
    double weight_phi_;
    double weight_v_;
    double weight_steeringFront_;
    double weight_steeringRear_;
    double weight_acceleration_;
    double weight_steeringFront_rate_;
    double weight_steeringRear_rate_;

    //kalman filter control
    bool kalman_;
};

// class for definition of difference equations and cost function in ipopt style
class FG_eval {
private:
    // vehicle parameters
    double m, J_z, lf, lr, stiffness, T_ax, T_steer;

    // control parameters
    double dt;// step time
    double weights[8];// penalties for different parts of cost function
    double y_soll[horizon_length], phi_soll[horizon_length], v_soll[horizon_length];// trajectory reference values

public:
    FG_eval(double cycle_t_, double* weights, double* y_ref, double* phi_ref, double* v_ref, double* params);
    typedef CPPAD_TESTVECTOR(CppAD::AD<double> )ADvector;
    void operator()(ADvector& fg, const ADvector& x);
};


#endif //SRC_MODEL_PREDICTIVE_CONTROLLER_NONLIN_H
