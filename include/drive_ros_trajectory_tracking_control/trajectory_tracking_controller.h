#ifndef DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H
#define DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <drive_ros_msgs/TrajectoryMetaInput.h>
#include <drive_ros_msgs/Trajectory.h>
#include <drive_ros_uavcan/phoenix_msgs__NucDriveCommand.h>
#include <drive_ros_uavcan/phoenix_msgs__DriveState.h>
#include <drive_ros_uavcan/phoenix_msgs__ImuData.h>
#include <drive_ros_trajectory_tracking_control/estimator_systemmodel.h>
#include <drive_ros_trajectory_tracking_control/estimator_measurementmodels.h>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <nav_msgs/Path.h>

class TrajectoryTrackingController {
public:
    TrajectoryTrackingController(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~TrajectoryTrackingController();
    // typedefs for Kalman-Filter
    typedef Models::State<double> State;
    typedef Models::Control<double> Control;
    typedef Models::SystemModel<double> SystemModel;
    typedef Models::VehicleStateMeasurement<double> VehicleState;
    typedef Models::ImuStateMeasurement<double> ImuMeasurement;
    typedef Models::VehicleStateMeasurementModel<double> StateModel;
    typedef Models::ImuStateMeasurementModel<double> ImuModel;
protected:
    // actually just temp until we get a proper trajectory generator
    void processMetaInput();
    void metaInputCB(const drive_ros_msgs::TrajectoryMetaInputConstPtr &msg);
    void driveStateCB(const drive_ros_uavcan::phoenix_msgs__DriveStateConstPtr &msg);
    void imuStateCB(const drive_ros_uavcan::phoenix_msgs__ImuDataConstPtr &msg);
    // this is where the control magic happens
    void trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg);
    drive_ros_msgs::TrajectoryPoint getTrajectoryPoint(
            const double distanceToPoint , const drive_ros_msgs::TrajectoryConstPtr &trajectory);
    // NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // subscribers
    ros::Subscriber meta_input_sub_;
    ros::Subscriber drive_state_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber imu_state_sub_;

    // publishers
    ros::Publisher nuc_command_pub_;
    ros::Publisher planned_path_pub;

    // params for system model
    const double l = 0.3302;// wheelbase
    const double stiffness = 5.0;// cornering stiffness
    const double lf = 0.15875;// distance CG - front tire
    const double lr = 0.17145;// distance CG -  rear tire
    const double m = 3.47;// mass
    const double J_z = 0.04712;// moment of inertia
    const double T_ax = 0.06;// acceleration time constant
    const double T_steer = 0.05;// steering time constant

    // store current driving state
    float cur_v_ = 0;
    float cur_angle_f_ = 0;
    float cur_angle_r_ = 0;
    float cur_yaw_ = 0;
    float cur_acc_ = 0;
    float cur_beta_ = 0;

    // kalman estimator
    State estimator_x;
    Control estimator_u;
    SystemModel estimator_sys;
    StateModel estimator_state_meas_model;
    ImuModel estimator_imu_meas_model;
    Kalman::UnscentedKalmanFilter<State> estimator_ukf;

    double link_length_;
    double max_longitudinal_acc_;
    double u_1_ub_;
    double u_1_lb_;
    double u_2_ub_;
    double u_2_lb_;
    double nodes_v_max_;
    double nodes_v_min_;
    double cycle_t_;
    double angle_bound;
    double angle_rate_bound;
    // store current meta inputs
    // todo: handle this stuff in trajectory generator, this should not be needed in the controller
    float v_max_ = 0;
    int driving_command_ = 0;
};


#endif //DRIVE_TRAJECTORY_TRACKING_CONTROLLER_H
