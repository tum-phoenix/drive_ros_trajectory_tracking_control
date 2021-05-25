//
// Created by sebastian on 02.07.19.
//

#ifndef SRC_MODEL_PREDICTIVE_CONTROLLER_NONLIN_H
#define SRC_MODEL_PREDICTIVE_CONTROLLER_NONLIN_H
#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>
#include <drive_ros_msgs/TrajectoryPoint.h>
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>



class ModelPredictiveController_nonlin : public TrajectoryTrackingController{
public:
    ModelPredictiveController_nonlin(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController_nonlin();
private:
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);
    std::string stream_name_ = "ModelPredictiveController_nonlin";

    // class for definition of difference equations and cost function in ipopt style
    class FG_eval {
    private:
        // vehicle parameters
        const double l = 0.2405;// wheelbase
        const double stiffness = 20.0;// cornering stiffness
        const double lf = 0.5*l;// distance CG front tire
        const double lr = 0.5*l;// distance CG rear tire
        const double m = 10;// mass
        const double J_z = 0.1;// moment of inertia
        const double T_ax = 0.5;// acceleration time constant
        const double T_steer = 0.1;// steering time constant

        // control parameters
        static const size_t N = 4;// horizon_length
        double dt;// step time
        double weights[8];// penalties for different parts of cost function
        double y_soll[N], phi_soll[N], v_soll[N];// trajectory reference values

        // index variables for states and controls
        size_t y_idx = 0;
        size_t phi_idx = N;
        size_t v_idx = phi_idx + N;
        size_t beta_idx = v_idx + N;
        size_t yaw_idx = beta_idx + N;
        size_t a_idx = yaw_idx + N;
        size_t delta_f_idx = a_idx + N;
        size_t delta_r_idx = delta_f_idx + N;
        size_t a_ref_idx = delta_r_idx + N;
        size_t delta_f_ref_idx = a_ref_idx + N - 1;
        size_t delta_r_ref_idx = delta_f_ref_idx + N -1;
    public:
        FG_eval(double cycle_t_, double* weight, double* y, double* phi, double* v);
        typedef CPPAD_TESTVECTOR(CppAD::AD<double> )ADvector;
        void operator()(ADvector& fg, const ADvector& x);
    };

    // control parameters
    double weight_y_;
    double weight_phi_;
    double weight_v_;
    double weight_steeringFront_;
    double weight_steeringRear_;
    double weight_acceleration_;
    double weight_steeringFront_rate_;
    double weight_steeringRear_rate_;
    size_t horizon_length=4;



};


#endif //SRC_MODEL_PREDICTIVE_CONTROLLER_NONLIN_H
