#include <drive_ros_trajectory_tracking_control/model_predictive_controller_nonlin.h>

FG_eval::FG_eval(double cycle_t_, double* weights, double* y_ref, double* phi_ref,
                                                   double* v_ref, double* params){
    this->dt = cycle_t_;
    for(size_t i=0; i<8; i++){
        this->weights[i] = weights[i];
    }
    for(size_t i=0; i<horizon_length; i++){
        this->y_soll[i] = y_ref[i];
        this->phi_soll[i] = phi_ref[i];
        this->v_soll[i] = v_ref[i];
    }
    this->m = params[0];
    this->J_z = params[1];
    this->lf = params[2];
    this->lr = params[3];
    this->stiffness = params[4];
    this->T_ax = params[5];
    this->T_steer = params[6];
}

void FG_eval::operator()(ADvector& fg, const ADvector& x)
{
    // Cost function
    fg[0]=0;

    // y error, phi error and v error
    for(size_t i=0; i<horizon_length; i++){
        fg[0] += weights[0] * CppAD::pow(x[y_idx+i] - y_soll[i], 2);
        fg[0] += weights[1] * CppAD::pow(x[phi_idx+i] - phi_soll[i], 2);
        fg[0] += weights[2] * CppAD::pow(x[v_idx+i] - v_soll[i], 2);
    }

    // Penalize high accelerations and steering angles
    for(size_t i=0; i<horizon_length-1; i++) {
        fg[0] += weights[3] * CppAD::pow(x[a_idx + i], 2);
        fg[0] += weights[4] * CppAD::pow(x[delta_f_ref_idx + i], 2);
        fg[0] += weights[5] * CppAD::pow(x[delta_r_ref_idx + i], 2);
    }

    // Penalize difference in steering
    for(size_t i=0; i<horizon_length-2; i++){
        fg[0] += weights[6] * CppAD::pow(x[delta_f_idx + i + 1] - x[delta_f_idx +i], 2);
        fg[0] += weights[7] * CppAD::pow(x[delta_r_idx + i + 1] - x[delta_f_idx + i], 2);
    }

    // initial states
    fg[1 + y_idx] = x[y_idx];
    fg[1 + phi_idx] = x[phi_idx];
    fg[1 + v_idx] = x[v_idx];
    fg[1 + beta_idx] = x[beta_idx];
    fg[1 + yaw_idx] = x[yaw_idx];
    fg[1 + a_idx] = x[a_idx];
    fg[1 + delta_f_idx] = x[delta_f_idx];
    fg[1 + delta_r_idx] = x[delta_r_idx];

    // model equations(single track model) for all points within the horizon
    for(size_t i=0; i<horizon_length-1; i++){
        // x_k, u_k
        CppAD::AD<double> y_old = x[y_idx + i];
        CppAD::AD<double> phi_old = x[phi_idx + i];
        CppAD::AD<double> v_old = x[v_idx + i];
        CppAD::AD<double> beta_old = x[beta_idx + i];
        CppAD::AD<double> yaw_old = x[yaw_idx + i];
        CppAD::AD<double> a_old = x[a_idx + i];
        CppAD::AD<double> delta_f_old = x[delta_f_idx + i];
        CppAD::AD<double> delta_r_old = x[delta_r_idx + i];
        CppAD::AD<double> a_ref_old = x[a_ref_idx + i];
        CppAD::AD<double> delta_f_ref_old = x[delta_f_ref_idx + i];
        CppAD::AD<double> delta_r_ref_old = x[delta_r_ref_idx + i];

        // tire forces
        CppAD::AD<double> F_lat_f = stiffness * (delta_f_old - beta_old - lf * yaw_old / v_old);
        CppAD::AD<double> F_lat_r = stiffness * (delta_r_old - beta_old + lr * yaw_old / v_old);
        CppAD::AD<double> F_lon = m * a_old / (CppAD::cos(delta_f_old) + CppAD::cos(delta_r_old));

        // difference equations for all states written as Constraints for Optimization
        fg[2 + y_idx + i] = - x[y_idx + i + 1] + y_old + v_old * CppAD::sin(beta_old + phi_old) * dt;
        fg[2 + phi_idx + i] = - x[phi_idx + i + 1] + phi_old + yaw_old * dt;
        fg[2 + v_idx + i] = - x[v_idx + i + 1] + v_old + (1 / m * (F_lon * CppAD::cos(delta_f_old - beta_old)
                + F_lon * CppAD::cos(delta_r_old - beta_old) - F_lat_f * CppAD::sin(delta_f_old - beta_old)
                - F_lat_r * CppAD::sin(delta_r_old - beta_old))) * dt;
        fg[2 + beta_idx + i] = - x[beta_idx + i + 1] + beta_old + (- yaw_old + 1 / (m * v_old) * (F_lon
                * CppAD::sin(delta_f_old - beta_old) + F_lon * CppAD::sin(delta_r_old - beta_old) + F_lat_f
                * CppAD::cos(delta_f_old - beta_old) + F_lat_r * CppAD::cos(delta_r_old - beta_old))) * dt;
        fg[2 + yaw_idx + i] = - x[yaw_idx + i + 1] + yaw_old + (1 / J_z * (lf * F_lon * CppAD::sin(delta_f_old) - lr
                * F_lon * CppAD::sin(delta_r_old) + lf * F_lat_f * CppAD::cos(delta_f_old) - lr * F_lat_r
                * CppAD::cos(delta_r_old))) * dt;
        fg[2 + a_idx + i] = - x[a_idx + i +1] + a_old + (1 / T_ax * (a_ref_old - a_old)) * dt;
        fg[2 + delta_f_idx + i] = - x[delta_f_idx + i +1] + delta_f_old + (1 / T_steer *
                (delta_f_ref_old - delta_f_old)) * dt;
        fg[2 + delta_r_idx + i] = - x[delta_r_idx + i +1] + delta_r_old + (1 / T_steer *
                (delta_r_ref_old - delta_r_old)) * dt;
    }
    return;
}


ModelPredictiveController_nonlin::ModelPredictiveController_nonlin(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh)
{
    //von config einlesen, um live einzustellen
    pnh_.getParam("penalty_y", weight_y_);
    pnh_.getParam("penalty_phi", weight_phi_);
    pnh_.getParam("penalty_v", weight_v_);
    pnh_.getParam("penalty_front_angle", weight_steeringFront_);
    pnh_.getParam("penalty_rear_angle", weight_steeringRear_);
    pnh_.getParam("penalty_acceleration", weight_acceleration_);
    pnh_.getParam("penalty_front_angle_rate", weight_steeringFront_rate_);
    pnh_.getParam("penalty_rear_angle_rate", weight_steeringRear_rate_);
    pnh_.getParam("kalman_filter_on", kalman_);

    trajectory_sub_ = nh_.subscribe("local_trajectory", 1, &ModelPredictiveController_nonlin::trajectoryCB, this);
}

ModelPredictiveController_nonlin::~ModelPredictiveController_nonlin() {}

void ModelPredictiveController_nonlin::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {
    if (kalman_){
        cur_v_ = estimator_x.v();
        cur_angle_f_ = estimator_x.delta_f();
        cur_angle_r_ = estimator_x.delta_r();
        cur_yaw_ = estimator_x.yaw();
        cur_acc_ = estimator_x.acc();
        cur_beta_ = estimator_x.beta();
    }

    ROS_INFO_STREAM("v: "<< cur_v_ <<"  beta: "<< cur_beta_ << "  yaw: "<< cur_yaw_);

    // check received trajectory
    if (msg->points.size() == 0) {
        ROS_ERROR_STREAM("MPC received empty trajectory, skipping!");
        return;
    }

    // check current velocity
    if(fabs(cur_v_) < 0.1){
        ROS_INFO_NAMED(stream_name_,"car is slow: ");
        cur_v_ = 0.1;//Some controller has some issue divides by v without error-checking
    }

    // get target points from trajectory

    // one target point for every prediction step
    double distance_to_trajectory_points[horizon_length+1];
    drive_ros_msgs::TrajectoryPoint target_points[horizon_length+1];
    for(int i=0; i<horizon_length+1; i++){
        distance_to_trajectory_points[i] = i * cur_v_ * cycle_t_;
        target_points[i] = getTrajectoryPoint(distance_to_trajectory_points[i], msg);
    }



    //set desired states

    // one target point per prediction step
    double phi_soll[horizon_length];
    double y_soll[horizon_length];
    double v_soll[horizon_length];
    for(int i=0; i<horizon_length; i++){
        phi_soll[i] = atan2(target_points[i+1].pose.y-target_points[i].pose.y,target_points[i+1].pose.x-target_points[i].pose.x);
        y_soll[i] = target_points[i].pose.y;
        v_soll[i] = sqrt(pow(target_points[i].twist.x,2)+pow(target_points[i].twist.y,2));
    }

    size_t i;
    typedef CPPAD_TESTVECTOR( double ) Dvector;

    // number of variables for optimization problem(states and controls for all timesteps)
    size_t nx = horizon_length * 8 + (horizon_length-1) * 3;
    // number of constraints(initial state and difference equations for all timesteps)
    size_t ng = horizon_length * 8;

    // initialize all variables to 0
    Dvector xi(nx);
    for(i=0; i<nx; i++){
        xi[i] = 0;
    }

    // initial states
    // xi[0] = 0.0; // y0=0
    // xi[horizon_length] = 0.0; // phi0=0
    xi[v_idx] = cur_v_; // v0
    xi[beta_idx] = cur_beta_; // beta0
    xi[yaw_idx] = cur_yaw_; // yaw0
    xi[a_idx] = cur_acc_; // acc0
    xi[delta_f_idx] = cur_angle_f_; // front_steering_angle0
    xi[delta_r_idx] = cur_angle_r_; // rear_steering_angle0

    // prevent division by 0(division by velocity within model equations)
    for(size_t i=1; i<horizon_length;i++) {
        xi[v_idx + i] = 0.1; // v
    }

    // state constraints
    Dvector xl(nx), xu(nx);
    for(i = 0; i < nx; i++)
    {
        xl[i] = -1e14;
        xu[i] = 1e14;
    }

    // constraints for acceleration, steer_f and steer_r
    double radian_bound = (M_PI/180)*angle_bound;
    for(i = 0; i < horizon_length; i++){
        xl[a_idx + i] = -max_longitudinal_acc_;
        xu[a_idx + i] = max_longitudinal_acc_;
        xl[delta_f_idx + i] = -radian_bound;
        xu[delta_f_idx + i] = radian_bound;
        xl[delta_r_idx + i] = -radian_bound;
        xu[delta_r_idx + i] = radian_bound;
    }

    // constraints for acceleration, steer_f and steer_r command
    for(i = 0; i < horizon_length-1; i++){
        xl[a_ref_idx + i] = -max_longitudinal_acc_;
        xu[a_ref_idx + i] = max_longitudinal_acc_;
        xl[delta_f_ref_idx + i] = -radian_bound;
        xu[delta_f_ref_idx + i] = radian_bound;
        xl[delta_r_ref_idx + i] = -radian_bound;
        xu[delta_r_ref_idx + i] = radian_bound;
    }

    // lower and upper limits for g (set to 0 for model equations)
    Dvector gl(ng), gu(ng);
    for(i = 0; i < ng; i++){
        gl[i] = 0;
        gu[i] = 0;
    }

    // initial state constraints
    //gl[0] = 0;// y0=0
    //gl[1] = 0;// phi0=0
    gl[v_idx] = cur_v_;// v0
    gl[beta_idx] = cur_beta_;// beta0
    gl[yaw_idx] = cur_yaw_;// yaw0
    gl[a_idx] = cur_acc_;// acc0
    gl[delta_f_idx] = cur_angle_f_;// front_steering_angle0
    gl[delta_r_idx] = cur_angle_r_;// rear_steering_angle0
    //gu[0] = 0;// y0=0
    //gu[1] = 0;// phi0=0
    gu[v_idx] = cur_v_;// v0
    gu[beta_idx] = cur_beta_;// beta0
    gu[yaw_idx] = cur_yaw_;// yaw0
    gu[a_idx] = cur_acc_;// acc0
    gu[delta_f_idx] = cur_angle_f_;// front_steering_angle0
    gu[delta_r_idx] = cur_angle_r_;// rear_steering_angle0

    // initialize class for cost function and constraints with weights and reference values
    double weights[] = {weight_y_, weight_phi_, weight_v_, weight_acceleration_, weight_steeringFront_,
                        weight_steeringRear_, weight_steeringFront_rate_, weight_steeringRear_rate_};
    double params[]={m, J_z, lf, lr, stiffness, T_ax, T_steer};
    FG_eval fg_eval(cycle_t_, weights, y_soll, phi_soll, v_soll, params);

    // options for optimization
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse true         reverse\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);

    // set control commands
    double control[3];
    control[0] = solution.x[a_ref_idx];
    control[1] = solution.x[delta_f_ref_idx];
    control[2] = solution.x[delta_r_ref_idx];
    control[0] = cur_v_ + control[0] * cycle_t_;
    if(std::isnan(control[1]) || std::isnan(control[2] || std::isnan(control[0])) ){
        ROS_ERROR_STREAM("invalid vals: " << control[1] <<" " <<control[2]<<" "<<control[0]);
    }

    //Set values
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command_msg;

    drive_command_msg.phi_f = control[1]; //publish as driving command
    drive_command_msg.phi_r = control[2];
    drive_command_msg.lin_vel = control[0];
    nuc_command_pub_.publish(drive_command_msg);
    ROS_INFO_NAMED(stream_name_, "Steering front = %.5f", drive_command_msg.phi_f * 180.f / M_PI);
    ROS_INFO_NAMED(stream_name_, "Steering rear = %.5f", drive_command_msg.phi_r * 180.f / M_PI);
    ROS_INFO_NAMED(stream_name_, "Velocity_command = %.5f", drive_command_msg.lin_vel);

    // kalman update step
    estimator_u.acc_ref() = (control[0] - cur_v_) / cycle_t_;
    estimator_u.delta_f_ref() = control[1];
    estimator_u.delta_r_ref() = control[2];
    estimator_x = estimator_ukf.predict(estimator_sys, estimator_u);
    return;
}