//
// Created by phillip on 23.09.21.
//

#ifndef SRC_MODEL_PREDICTIVE_CONTROLLER_ACADOS_H
#define SRC_MODEL_PREDICTIVE_CONTROLLER_ACADOS_H

#include <ros/ros.h>
#include <drive_ros_trajectory_tracking_control/trajectory_tracking_controller.h>
#include <drive_ros_msgs/TrajectoryPoint.h>
#include <cmath>
#include <assert.h>
#include <stdlib.h>

// model
#include "drive_ros_trajectory_tracking_control/model/single_track_model_impl_ode_fun.h"
#include "drive_ros_trajectory_tracking_control/model/single_track_model_impl_ode_fun_jac_x_xdot.h"
#include "drive_ros_trajectory_tracking_control/model/single_track_model_impl_ode_fun_jac_x_xdot_u.h"
#include "drive_ros_trajectory_tracking_control/model/single_track_model_impl_ode_jac_x_xdot_u.h"


// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"
#include "blasfeo/include/blasfeo_i_aux_ext_dep.h"

// acados
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados/utils/mem.h"
#include "acados/utils/print.h"
#include "acados/utils/timing.h"
#include "acados/utils/types.h"



class ModelPredictiveController_acados : public TrajectoryTrackingController{
public:
    ModelPredictiveController_acados(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController_acados();
private:
    static void select_dynamics_single_track_casadi(int N,
            external_function_param_casadi *impl_ode_fun,
            external_function_param_casadi *impl_ode_fun_jac_x_xdot,
            external_function_param_casadi *impl_ode_jac_x_xdot_u,
            external_function_param_casadi *impl_ode_fun_jac_x_xdot_u);
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);
    std::string stream_name_ = "ModelPredictiveController_acados";

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

    // horizon_length
    int horizon_length;
};


#endif //SRC_MODEL_PREDICTIVE_CONTROLLER_ACADOS_H
