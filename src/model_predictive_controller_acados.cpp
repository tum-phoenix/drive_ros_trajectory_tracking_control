//
// Created by phillip on 23.09.21.
//

#include <drive_ros_trajectory_tracking_control/model_predictive_controller_acados.h>



ModelPredictiveController_acados::ModelPredictiveController_acados(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh)
{
    //von config einlesen, um live einzustellen
    pnh_.getParam("mpc_acados/penalty_y", weight_y_);
    pnh_.getParam("mpc_acados/penalty_phi", weight_phi_);
    pnh_.getParam("mpc_acados/penalty_v", weight_v_);
    pnh_.getParam("mpc_acados/penalty_front_angle", weight_steeringFront_);
    pnh_.getParam("mpc_acados/penalty_rear_angle", weight_steeringRear_);
    pnh_.getParam("mpc_acados/penalty_acceleration", weight_acceleration_);
    pnh_.getParam("mpc_acados/penalty_front_angle_rate", weight_steeringFront_rate_);
    pnh_.getParam("mpc_acados/penalty_rear_angle_rate", weight_steeringRear_rate_);
    pnh_.getParam("kalman/kalman_filter_on", kalman_);
    pnh_.getParam("mpc_acados/horizon_length", horizon_length);

    trajectory_sub_ = nh_.subscribe("local_trajectory", 1, &ModelPredictiveController_acados::trajectoryCB, this);
}

ModelPredictiveController_acados::~ModelPredictiveController_acados() {}

void ModelPredictiveController_acados::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {
    if (kalman_){
        cur_v_ = estimator_x.v();
        cur_angle_f_ = estimator_x.delta_f();
        cur_angle_r_ = estimator_x.delta_r();
        cur_yaw_ = estimator_x.yaw();
        cur_acc_ = estimator_x.acc();
        cur_beta_ = estimator_x.beta();
    }

//    ROS_INFO_STREAM("front: "<< cur_angle_f_ * 180.f/M_PI<<"  back: "<< cur_angle_r_ * 180.f/M_PI<<"  v: "<< cur_v_);

    // check received trajectory
    if (msg->points.size() == 0) {
        ROS_ERROR_STREAM("MPC received empty trajectory, skipping!");
        return;
    }

    // check current velocity
    if(fabs(cur_v_) < 0.01){
        ROS_INFO_NAMED(stream_name_,"car is slow: ");
        cur_v_ = 0.01;//Some controller has some issue divides by v without error-checking
    }

    // get target points from trajectory

    // one target point for every prediction step
    double distance_to_trajectory_points[horizon_length+2];
    drive_ros_msgs::TrajectoryPoint target_points[horizon_length+2];
    for(int i=0; i<=horizon_length+1; i++){
        distance_to_trajectory_points[i] = i * cur_v_ * cycle_t_;
        target_points[i] = getTrajectoryPoint(distance_to_trajectory_points[i], msg);
    }
    /* plan + config*/
    ocp_nlp_plan *plan = ocp_nlp_plan_create(horizon_length);

    plan->nlp_solver = SQP;

    plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;

    for (int i = 0; i < horizon_length; i++)
    {
        plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        plan->sim_solver_plan[i].sim_solver = IRK;
        plan->nlp_cost[i] = LINEAR_LS;
        plan->nlp_constraints[i] = BGH;
    }
    plan->nlp_cost[horizon_length] = LINEAR_LS;
    plan->nlp_constraints[horizon_length] = BGH;

    ocp_nlp_config *config = ocp_nlp_config_create(*plan);

    /* dimensions */
    int nx_ = 10;
    int nu_ = 3;
    int ny_ = 8;
    int np = 1;
    // optimization variables
    int nx[horizon_length+1] = {}; // states
    int nu[horizon_length+1] = {}; // inputs
    int nz[horizon_length+1] = {}; // algebraic variables
    int ns[horizon_length+1] = {}; // slacks
    // cost
    int ny[horizon_length+1] = {}; // measurements
    // constraints
    int nbx[horizon_length+1] = {}; // state bounds
    int nbu[horizon_length+1] = {}; // input bounds
    int ng[horizon_length+1] = {}; // general linear constraints
    int nh[horizon_length+1] = {}; // nonlinear constraints
    int nsh[horizon_length+1] = {}; // softed nonlinear constraints

    // initial stage
    nx[0] = nx_;
    nu[0] = nu_;
    nbx[0] = nx_;
    nbu[0] = nu_;
    ng[0] = 0;
    nh[0] = 0;
    nsh[0] = 0;
    ns[0] = 0;
    ny[0] = ny_;
    nz[0] = 0;

    // middle stages
    for (int i = 1; i < horizon_length; i++)
    {
        nx[i] = nx_;
        nu[i] = nu_;
        nbx[i] = 5;
        nbu[i] = nu_;
        ng[i] = 0;
        nh[i] = 0;
        nsh[i] = 0;
        ns[i] = 0;
        ny[i] = ny_;
        nz[i] = 0;
    }

    // final stage
    nx[horizon_length] = nx_;
    nu[horizon_length] = 0;
    nbx[horizon_length] = 5;
    nbu[horizon_length] = 0;
    ng[horizon_length] = 0;
    nh[horizon_length] = 0;
    nsh[horizon_length] = 0;
    ns[horizon_length] = 0;
    ny[horizon_length] = 5;
    nz[horizon_length] = 0;

    /* ocp_nlp_dims */
    ocp_nlp_dims *dims = ocp_nlp_dims_create(config);

    ocp_nlp_dims_set_opt_vars(config, dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(config, dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(config, dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(config, dims, "ns", ns);

    for (int i = 0; i <= horizon_length; i++)
    {
        ocp_nlp_dims_set_cost(config, dims, i, "ny", &ny[i]);

        ocp_nlp_dims_set_constraints(config, dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(config, dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(config, dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(config, dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(config, dims, i, "nsh", &nsh[i]);
    }

    /* dynamics*/
    // implicit model
    external_function_param_casadi *impl_ode_fun = new external_function_param_casadi[horizon_length*sizeof(external_function_param_casadi)];
    external_function_param_casadi *impl_ode_fun_jac_x_xdot = new external_function_param_casadi[horizon_length*sizeof(external_function_param_casadi)];
    external_function_param_casadi *impl_ode_jac_x_xdot_u = new external_function_param_casadi[horizon_length*sizeof(external_function_param_casadi)];
    external_function_param_casadi *impl_ode_fun_jac_x_xdot_u = new external_function_param_casadi[horizon_length*sizeof(external_function_param_casadi)];

    for (int ii = 0; ii < horizon_length; ii++) {
        impl_ode_fun[ii].casadi_fun = &single_track_model_impl_ode_fun;
        impl_ode_fun[ii].casadi_work = &single_track_model_impl_ode_fun_work;
        impl_ode_fun[ii].casadi_sparsity_in = &single_track_model_impl_ode_fun_sparsity_in;
        impl_ode_fun[ii].casadi_sparsity_out = &single_track_model_impl_ode_fun_sparsity_out;
        impl_ode_fun[ii].casadi_n_in = &single_track_model_impl_ode_fun_n_in;
        impl_ode_fun[ii].casadi_n_out = &single_track_model_impl_ode_fun_n_out;

        impl_ode_fun_jac_x_xdot[ii].casadi_fun = &single_track_model_impl_ode_fun_jac_x_xdot;
        impl_ode_fun_jac_x_xdot[ii].casadi_work = &single_track_model_impl_ode_fun_jac_x_xdot_work;
        impl_ode_fun_jac_x_xdot[ii].casadi_sparsity_in = &single_track_model_impl_ode_fun_jac_x_xdot_sparsity_in;
        impl_ode_fun_jac_x_xdot[ii].casadi_sparsity_out = &single_track_model_impl_ode_fun_jac_x_xdot_sparsity_out;
        impl_ode_fun_jac_x_xdot[ii].casadi_n_in = &single_track_model_impl_ode_fun_jac_x_xdot_n_in;
        impl_ode_fun_jac_x_xdot[ii].casadi_n_out = &single_track_model_impl_ode_fun_jac_x_xdot_n_out;

        impl_ode_jac_x_xdot_u[ii].casadi_fun = &single_track_model_impl_ode_jac_x_xdot_u;
        impl_ode_jac_x_xdot_u[ii].casadi_work = &single_track_model_impl_ode_jac_x_xdot_u_work;
        impl_ode_jac_x_xdot_u[ii].casadi_sparsity_in = &single_track_model_impl_ode_jac_x_xdot_u_sparsity_in;
        impl_ode_jac_x_xdot_u[ii].casadi_sparsity_out = &single_track_model_impl_ode_jac_x_xdot_u_sparsity_out;
        impl_ode_jac_x_xdot_u[ii].casadi_n_in = &single_track_model_impl_ode_jac_x_xdot_u_n_in;
        impl_ode_jac_x_xdot_u[ii].casadi_n_out = &single_track_model_impl_ode_jac_x_xdot_u_n_out;

        impl_ode_fun_jac_x_xdot_u[ii].casadi_fun = &single_track_model_impl_ode_fun_jac_x_xdot_u;
        impl_ode_fun_jac_x_xdot_u[ii].casadi_work = &single_track_model_impl_ode_fun_jac_x_xdot_u_work;
        impl_ode_fun_jac_x_xdot_u[ii].casadi_sparsity_in = &single_track_model_impl_ode_fun_jac_x_xdot_u_sparsity_in;
        impl_ode_fun_jac_x_xdot_u[ii].casadi_sparsity_out = &single_track_model_impl_ode_fun_jac_x_xdot_u_sparsity_out;
        impl_ode_fun_jac_x_xdot_u[ii].casadi_n_in = &single_track_model_impl_ode_fun_jac_x_xdot_u_n_in;
        impl_ode_fun_jac_x_xdot_u[ii].casadi_n_out = &single_track_model_impl_ode_fun_jac_x_xdot_u_n_out;
    }
    external_function_param_casadi_create_array(horizon_length, impl_ode_fun, np);
    external_function_param_casadi_create_array(horizon_length, impl_ode_fun_jac_x_xdot, np);
    external_function_param_casadi_create_array(horizon_length, impl_ode_jac_x_xdot_u, np);
    external_function_param_casadi_create_array(horizon_length, impl_ode_fun_jac_x_xdot_u, np);

    /* nlp_in */
    ocp_nlp_in *nlp_in = ocp_nlp_in_create(config, dims);

    // sampling times
    for (int ii=0; ii<horizon_length; ii++)
    {
        nlp_in->Ts[ii] = cycle_t_;
    }

    /* dynamics */
    int set_fun_status;

    for (int i=0; i<horizon_length; i++)
    {
        set_fun_status = ocp_nlp_dynamics_model_set(config, dims, nlp_in, i, "impl_ode_fun", &impl_ode_fun[i]);
        if (set_fun_status != 0) exit(1);
        set_fun_status = ocp_nlp_dynamics_model_set(config, dims, nlp_in, i, "impl_ode_fun_jac_x_xdot", &impl_ode_fun_jac_x_xdot[i]);
        if (set_fun_status != 0) exit(1);
        set_fun_status = ocp_nlp_dynamics_model_set(config, dims, nlp_in, i, "impl_ode_jac_x_xdot_u", &impl_ode_jac_x_xdot_u[i]);
        if (set_fun_status != 0) exit(1);
    }


    /* linear least squares */
    // output definition
    // y = {x[0], x[1]; x[2] u[0]; u[1]; u[2]};
    //   = Vx * x + Vu * u

    double *Vx = new double[(ny_*nx_)*sizeof(double)];
    for (int ii=0; ii<ny_*nx_; ii++)
        Vx[ii] = 0.0;
    Vx[0+ny_*0] = 1.0;
    Vx[1+ny_*1] = 1.0;
    Vx[2+ny_*2] = 1.0;
    Vx[4+ny_*6] = 1.0;
    Vx[5+ny_*7] = 1.0;

    double *Vu = new double[(ny_*nu_)*sizeof(double)];
    for (int ii=0; ii<ny_*nu_; ii++)
        Vu[ii] = 0.0;
    Vu[3+ny_*0] = 1.0;
    Vu[6+ny_*1] = 1.0;
    Vu[7+ny_*2] = 1.0;

    double *VxN = new double[(ny[horizon_length]*nx[horizon_length])*sizeof(double)];
    for (int ii=0; ii<ny[horizon_length]*nx[horizon_length]; ii++)
        VxN[ii] = 0.0;
    VxN[0+ny[horizon_length]*0] = 1.0;
    VxN[1+ny[horizon_length]*1] = 1.0;
    VxN[2+ny[horizon_length]*2] = 1.0;
    VxN[3+ny[horizon_length]*6] = 1.0;
    VxN[4+ny[horizon_length]*7] = 1.0;

    double *W = new double[(ny_*ny_)*sizeof(double)];
    for (int ii=0; ii<ny_*ny_; ii++)
        W[ii] = 0.0;
    W[0+ny_*0] = weight_y_;
    W[1+ny_*1] = weight_phi_;
    W[2+ny_*2] = weight_v_;
    W[3+ny_*3] = weight_acceleration_;
    W[4+ny_*4] = weight_steeringFront_;
    W[5+ny_*5] = weight_steeringRear_;
    W[6+ny_*6] = weight_steeringFront_rate_;
    W[7+ny_*7] = weight_steeringRear_rate_;

    double *W_N = new double[(ny[horizon_length]*ny[horizon_length])*sizeof(double)];
    W_N[0+ny[horizon_length]*0] = weight_y_;
    W_N[1+ny[horizon_length]*1] = weight_phi_;
    W_N[2+ny[horizon_length]*2] = weight_v_;
    W_N[4+ny[horizon_length]*4] = weight_steeringFront_;
    W_N[5+ny[horizon_length]*5] = weight_steeringRear_;

    /* cost */

    // linear ls
    int status = ACADOS_SUCCESS;

    for (int i = 0; i <= horizon_length; i++) {
        // Cyt
        ocp_nlp_cost_model_set(config, dims, nlp_in, horizon_length, "Vu", Vu);
        if(i<horizon_length)ocp_nlp_cost_model_set(config, dims, nlp_in, i, "Vx", Vx);
        else ocp_nlp_cost_model_set(config, dims, nlp_in, i, "Vx", VxN);
        ocp_nlp_cost_model_set(config, dims, nlp_in, i, "Vu", Vu);
        ocp_nlp_cost_model_set(config, dims, nlp_in, i, "Vx", Vx);
        // W
        ocp_nlp_cost_model_set(config, dims, nlp_in, i, "W", W);
    }

    status = ocp_nlp_cost_model_set(config, dims, nlp_in, horizon_length, "W", W_N);

    /* bounds */
    double radian_bound = (M_PI/180)*angle_bound;
    // input bounds
    // initial state
    int *idxbu0 = new int[nbu[0]*sizeof(int)];
    double *lbu0 = new double[(nbu[0])*sizeof(double)];
    double *ubu0 = new double[(nbu[0])*sizeof(double)];

    idxbu0[0] = 0;
    lbu0[0] = -max_longitudinal_acc_;
    ubu0[0] = max_longitudinal_acc_;
    idxbu0[1] = 1;
    lbu0[1] = -angle_rate_bound;
    ubu0[1] = angle_rate_bound;
    idxbu0[2] = 2;
    lbu0[2] = -angle_rate_bound;
    ubu0[2] = angle_rate_bound;

    // middle stages
    int *idxbu1 = new int[nbu[1]*sizeof(int)];
    double *lbu1 = new double[(nbu[1])*sizeof(double)];
    double *ubu1 = new double[(nbu[1])*sizeof(double)];

    idxbu1[0] = 0;
    lbu1[0] = -max_longitudinal_acc_;
    ubu1[0] = max_longitudinal_acc_;
    idxbu1[1] = 1;
    lbu1[1] = -angle_rate_bound;
    ubu1[1] = angle_rate_bound;
    idxbu1[2] = 2;
    lbu1[2] = -angle_rate_bound;
    ubu1[2] = angle_rate_bound;

    // state bounds
    //inital state
    int *idxbx0 = new int[nbx[0]*sizeof(int)];
    double *lbx0 = new double[(nbx[0])*sizeof(double)];
    double *ubx0 = new double[(nbx[0])*sizeof(double)];

    // dummy
    for (int ii=0; ii<nbx[0]; ii++)
    {
        idxbx0[ii] = ii;
        lbx0[ii] = -1e8;
        ubx0[ii] = 1e8;;
    }
    // middle stages
    int *idxbx1 = new int[nbx[1]*sizeof(int)];
    double *lbx1 = new double[(nbx[1])*sizeof(double)];
    double *ubx1 = new double[(nbx[1])*sizeof(double)];

    idxbx1[0] = 5;
    lbx1[0] = -max_longitudinal_acc_;
    ubx1[0] = max_longitudinal_acc_;
    idxbx1[1] = 6;
    lbx1[1] = -radian_bound;
    ubx1[1] = radian_bound;
    idxbx1[2] = 7;
    lbx1[2] = -radian_bound;
    ubx1[2] = radian_bound;
    idxbx1[3] = 8;
    lbx1[3] = -angle_rate_bound;
    ubx1[3] = angle_rate_bound;
    idxbx1[4] = 9;
    lbx1[4] = -angle_rate_bound;
    ubx1[4] = angle_rate_bound;

    // last stage
    int *idxbxN = new int[nbx[horizon_length]*sizeof(int)];
    double *lbxN = new double[(nbx[horizon_length])*sizeof(double)];
    double *ubxN = new double[(nbx[horizon_length])*sizeof(double)];

    idxbxN[0] = 5;
    lbxN[0] = -max_longitudinal_acc_;
    ubxN[0] = max_longitudinal_acc_;
    idxbxN[1] = 6;
    lbxN[1] = -radian_bound;
    ubxN[1] = radian_bound;
    idxbxN[2] = 7;
    lbxN[2] = -radian_bound;
    ubxN[2] = radian_bound;
    idxbxN[3] = 8;
    lbxN[3] = -angle_rate_bound;
    ubxN[3] = angle_rate_bound;
    idxbxN[4] = 9;
    lbxN[4] = -angle_rate_bound;
    ubxN[4] = angle_rate_bound;

    /* constraints */
    // fist stage
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "idxbu", idxbu0);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "lbu", lbu0);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "ubu", ubu0);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "ubx", ubx0);
    // middle stages
    for (int i = 1; i < horizon_length; i++)
    {
        ocp_nlp_constraints_model_set(config, dims, nlp_in, i, "idxbu", idxbu1);
        ocp_nlp_constraints_model_set(config, dims, nlp_in, i, "lbu", lbu1);
        ocp_nlp_constraints_model_set(config, dims, nlp_in, i, "ubu", ubu1);
        ocp_nlp_constraints_model_set(config, dims, nlp_in, i, "idxbx", idxbx1);
        ocp_nlp_constraints_model_set(config, dims, nlp_in, i, "lbx", lbx1);
        ocp_nlp_constraints_model_set(config, dims, nlp_in, i, "ubx", ubx1);
    }
    // last stage
    ocp_nlp_constraints_model_set(config, dims, nlp_in, horizon_length, "idxbx", idxbxN);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, horizon_length, "lbx", lbxN);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, horizon_length, "ubx", ubxN);

    /* sqp opts*/
    // create opts
    void *nlp_opts = ocp_nlp_solver_opts_create(config, dims);

    // nlp opts

        int max_iter = 10;
        double tol_stat = 5e-2;
        double tol_eq   = 5e-3;
        double tol_ineq = 5e-3;
        double tol_comp = 5e-3;

        ocp_nlp_solver_opts_set(config, nlp_opts, "max_iter", &max_iter);
        ocp_nlp_solver_opts_set(config, nlp_opts, "tol_stat", &tol_stat);
        ocp_nlp_solver_opts_set(config, nlp_opts, "tol_eq", &tol_eq);
        ocp_nlp_solver_opts_set(config, nlp_opts, "tol_ineq", &tol_ineq);
        ocp_nlp_solver_opts_set(config, nlp_opts, "tol_comp", &tol_comp);

    // sim opts
    for (int i = 0; i < horizon_length; ++i)
    {
            int num_steps = horizon_length;
            int ns = 4;
            bool jac_reuse = true;

            ocp_nlp_solver_opts_set_at_stage(config, nlp_opts, i, "dynamics_num_steps", &num_steps);
            ocp_nlp_solver_opts_set_at_stage(config, nlp_opts, i, "dynamics_ns", &ns);
            ocp_nlp_solver_opts_set_at_stage(config, nlp_opts, i, "dynamics_jac_reuse", &jac_reuse);
    }
    // update opts after manual changes
    ocp_nlp_solver_opts_update(config, dims, nlp_opts);


    /* ocp_nlp_out + solver */
    ocp_nlp_out *nlp_out = ocp_nlp_out_create(config, dims);

    ocp_nlp_solver *solver = ocp_nlp_solver_create(config, dims, nlp_opts);

    // initial state
    double *specific_x = new double[nx_*sizeof(double)];
    double *specific_u = new double[nu_*sizeof(double)];
    specific_x[0] = 0.0;
    specific_x[1] = 0.0;
    specific_x[2] = cur_v_;
    specific_x[3] = cur_beta_;
    specific_x[4] = cur_yaw_;
    specific_x[5] = cur_acc_;
    specific_x[6] = cur_angle_f_;
    specific_x[7] = cur_angle_r_;
    specific_x[8] = 0.0;
    specific_x[9] = 0.0;
    specific_u[0] = 0.0;
    specific_u[1] = 0.0;
    specific_u[2] = 0.0;
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "lbx", specific_x);
    ocp_nlp_constraints_model_set(config, dims, nlp_in, 0, "ubx", specific_x);

    // warm start output
    for(int i=0; i<horizon_length; i++){
        ocp_nlp_out_set(config, dims, nlp_out, i, "u", specific_u);
        ocp_nlp_out_set(config, dims, nlp_out, i, "x", specific_x);
    }
    ocp_nlp_out_set(config, dims, nlp_out, horizon_length, "x", specific_x);

    /* precomputation */
    status = ocp_nlp_precompute(solver, nlp_in, nlp_out);

    /* solve */

    // update reference
    //set desired states

    // one target point per prediction step
    double *y_ref = new double[ny_*(horizon_length+1)*sizeof(double)];
    for(int i=0; i<=horizon_length; i++){
        y_ref[i*ny_] = target_points[i].pose.y;
        y_ref[1+i*ny_] = atan2(target_points[i+1].pose.y-target_points[i].pose.y,target_points[i+1].pose.x-target_points[i].pose.x);
        y_ref[2+i*ny_] = sqrt(pow(target_points[i].twist.x,2)+pow(target_points[i].twist.y,2));
        y_ref[3+i*ny_] = 0.0;
        y_ref[4+i*ny_] = 0.0;
        y_ref[5+i*ny_] = 0.0;
        y_ref[6+i*ny_] = 0.0;
        y_ref[7+i*ny_] = 0.0;
    }
    for (int i = 0; i <= horizon_length; i++)
    {
        ocp_nlp_cost_model_set(config, dims, nlp_in, i, "yref", &y_ref[i*ny_]);
    }

    //solve
    status = ocp_nlp_solve(solver, nlp_in, nlp_out);

    nav_msgs::Path path;
    path.header.frame_id = "vehicle_frame";
    geometry_msgs::PoseStamped pose;
    for(int i=0; i<horizon_length; i++){
        pose.header.frame_id = "vehicle_frame";
        pose.pose.position.x = i * cur_v_ * cycle_t_;
        pose.pose.position.y = y_ref[i*ny_];
        path.poses.push_back(pose);
    }
    planned_path_pub.publish(path);

    // set control commands
    double *control = new double[nu_*sizeof(double)];
    ocp_nlp_out_get(config, dims, nlp_out, 0, "u", control);
    control[0] = cur_v_ + control[0] * cycle_t_;
    control[1] = cur_angle_f_ + control[1] * cycle_t_;
    control[2] = cur_angle_r_ + control[2] * cycle_t_;
    if(std::isnan(control[1]) || std::isnan(control[2] || std::isnan(control[0])) ){
        ROS_ERROR_STREAM("invalid vals: " << control[1] <<" " <<control[2]<<" "<<control[0]);
    }

    //Set message
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command_msg;

    drive_command_msg.lin_vel = control[0];//publish as driving command
    drive_command_msg.phi_f = control[1];
    drive_command_msg.phi_r = control[2];
    nuc_command_pub_.publish(drive_command_msg);
    ROS_INFO("Com:front=%.3f, back=%.3f, speed=%.3f ", drive_command_msg.phi_f * 180.f / M_PI, drive_command_msg.phi_r * 180.f / M_PI, drive_command_msg.lin_vel);

    // kalman update step
    estimator_u.acc_ref() = (control[0]-cur_v_)/cycle_t_;
    estimator_u.delta_f_ref() = control[1];
    estimator_u.delta_r_ref() = control[2];
    estimator_x = estimator_ukf.predict(estimator_sys, estimator_u);
    return;
}