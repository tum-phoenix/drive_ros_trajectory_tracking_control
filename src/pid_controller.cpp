

#include "../include/drive_ros_trajectory_tracking_control/pid_controller.h"

PIDController::PIDController(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh) {

    trajectory_sub_ = nh.subscribe("local_trajectory", 1, &PIDController::trajectoryCB, this);
    pnh_.getParam("pid_k_p",k_p);
    pnh_.getParam("pid_k_i",k_i);
    pnh_.getParam("pid_k_d",k_d);
    pnh_.getParam("k_p_vel", k_p_vel);
    pnh_.getParam("k_i_vel", k_i_vel);
    pnh_.getParam("k_d_vel", k_d_vel);
    esum=0;
    eold=0;
}


PIDController::~PIDController() {}

void PIDController::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {
    if (msg->points.size() == 0) {
        ROS_ERROR_STREAM("PID received empty trajectory, skipping!");
        return;
    }
    // for simulation just use velocity of first trajectory point
    //double v=msg->points[0].twist.x;

    double v=cur_v_;

    //check velocity
    if(fabs(v) < 0.1){
        ROS_INFO_NAMED(stream_name_, "car is slow: ");
        v=0.1;//Some controller has some issue divides by v without error-checking
    }
    // taking a point in front of the car improves stability
    double distance_to_trajectory_point = 2 * v * cycle_t_;
    drive_ros_msgs::TrajectoryPoint target_point=getTrajectoryPoint(distance_to_trajectory_point,msg);

    auto pid_x = target_point.pose.x;
    auto pid_y = target_point.pose.y;
    auto pid_vx = target_point.twist.x;
    auto pid_vy = target_point.twist.y;



    //calculate steering angle

    //control error
    float e=pid_y;
    //integrated error
    esum+=e;

    // gain
    float kappa = k_p*e+k_d*(e-eold)+k_i*esum;

    //update control errors
    eold=e;

    //saturation
    if (kappa>(angle_bound*M_PI/180.f)){
        kappa=angle_bound*M_PI/180.f;
    }
    else if(kappa<(-angle_bound*M_PI/180.f)){
        kappa=-angle_bound*M_PI/180.f;
    }

    //print goal point and steering angle
    ROS_INFO_NAMED(stream_name_, "Goal point (%.2f, %.2f)", pid_x, pid_y);
    ROS_INFO_NAMED(stream_name_, "Kappa = %.5f", kappa);

    // velocity PID-controller
    double e_vel = pid_vx-v;
    e_sum_vel += e_vel * cycle_t_;
    double v_desired = k_p_vel * e_vel + k_i_vel * e_sum_vel + k_d_vel * (e_vel - e_old_vel) / cycle_t_;
    e_old_vel = e_vel;
    if (v_desired>5){
        v_desired=5;
    }
    if(v_desired<-5){
        v_desired=-5;
    }


    ROS_INFO_NAMED(stream_name_, "vGoal = %f", pid_vx);

    drive_ros_uavcan::phoenix_msgs__NucDriveCommand::_blink_com_type blink_com =
            drive_ros_uavcan::phoenix_msgs__NucDriveCommand::NO_BLINK;

    drive_ros_uavcan::phoenix_msgs__NucDriveCommand driveCmdMsg;
    //kappa=-kappa; //for simulation not necessary
    driveCmdMsg.phi_f = kappa;
    driveCmdMsg.phi_r = 0.f;
    driveCmdMsg.lin_vel = v_desired;
    driveCmdMsg.blink_com = blink_com;

    //if(!isnanf(steeringAngleFront) && !isnanf(steeringAngleRear)) {
    ROS_INFO_NAMED(stream_name_, "Steering front = %.1f[deg]", driveCmdMsg.phi_f * 180.f / M_PI);
    ROS_INFO_NAMED(stream_name_, "Steering rear  = %.1f[deg]", driveCmdMsg.phi_r * 180.f / M_PI);

    nuc_command_pub_.publish(driveCmdMsg);
    ROS_INFO_NAMED(stream_name_, "Published uavcan message");

}
