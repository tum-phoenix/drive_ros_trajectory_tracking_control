
#include <drive_ros_trajectory_tracking_control/model_predictive_controller_dlib.h>


ModelPredictiveController_dlib::ModelPredictiveController_dlib(ros::NodeHandle nh, ros::NodeHandle pnh):
        TrajectoryTrackingController(nh, pnh)
{

    pnh_.getParam("penalty_y", weight_y_);
    pnh_.getParam("penalty_phi", weight_phi_);
    pnh_.getParam("penalty_front_angle", weight_steeringFront_);
    pnh_.getParam("penalty_rear_angle", weight_steeringRear_);
    //von config einlesen, um live einzustellen


    trajectory_sub_ = nh.subscribe("local_trajectory", 1, &ModelPredictiveController_dlib::trajectoryCB, this);
}

ModelPredictiveController_dlib::~ModelPredictiveController_dlib() {}

void ModelPredictiveController_dlib::trajectoryCB(const drive_ros_msgs::TrajectoryConstPtr &msg) {
    // for simulation velocity from first trajectory point
    //double v =cur_v_
    if (msg->points.size() == 0) {
        ROS_ERROR_STREAM("MPC received empty trajectory, skipping!");
        return;
    }
    double v=msg->points[0].twist.x;

    //check velocity
    if(fabs(v) < 0.1){
        ROS_INFO_NAMED(stream_name_,"car is slow: ");
        v=0.1;//Some controller has some issue divides by v without error-checking
    }

    // get target points from trajectory
    // velocity from first trajectory point

    //implementation from lms with just one target point that is applied to all prediction steps
    //double distance_to_trajectory_point;
    //drive_ros_msgs::TrajectoryPoint target_point;
    //    distance_to_trajectory_point=v*cycle_t_+0.4;
    //    target_point=getTrajectoryPoint(distance_to_trajectory_point,msg);

    // one target point for every prediction step
    double distance_to_trajectory_points[horizon_length+1];
    drive_ros_msgs::TrajectoryPoint target_points[horizon_length+1];
    for(int i=0; i<horizon_length+1; i++){
        distance_to_trajectory_points[i]=(i+1)*v*cycle_t_;
        target_points[i]=getTrajectoryPoint(distance_to_trajectory_points[i],msg);
    }



    //set desired states

    //one target point
//    double phi_soll;
//    double y_soll;
//    phi_soll=atan2(target_point.pose.y,target_point.pose.x);
//    y_soll=target_point.pose.y;
//    ROS_INFO_NAMED(stream_name_,"phi y: %.5f, %.5f ", phi_soll, y_soll);

    //one target point per prediction step
    double phi_soll[horizon_length];
    double y_soll[horizon_length];
    for(int i=0; i<horizon_length; i++){
        phi_soll[i]=atan2(target_points[i+1].pose.y-target_points[i].pose.y,target_points[i+1].pose.x-target_points[i].pose.x);
        y_soll[i]=target_points[i].pose.y;
        //ROS_INFO_NAMED(stream_name_,"Schritt: %d: phi y: %.5f, %.5f ",i+1, phi_soll[i], y_soll[i]);
    }




    double steering_front, steering_rear;
    const int STATES_ = 2; //number of states (y and phi)
    const int CONTROLS_ = 2; //number of control inputs (steering_front and steering_rear)
    //declare input bounds
    double radian_bound = (M_PI/180)*angle_bound;
    dlib::matrix<double,CONTROLS_,1> lower, upper;
    lower= -radian_bound, -radian_bound;
    upper= radian_bound, radian_bound;

    dlib::matrix<double,STATES_,STATES_> A;
    A = 1, cycle_t_*v, 0, 1;

    dlib::matrix<double,STATES_,CONTROLS_> B;
    B = 0, 0, cycle_t_*v/l, -cycle_t_*v/l;

    dlib::matrix<double,STATES_,1> C; //keine konstante Stoerung
    C = 0, 0;

    dlib::matrix<double,STATES_,1> Q; //Gewichtung der Regelabweichung fuer Querabstand y und Orientierung phi
    Q = weight_y_, weight_phi_;

    dlib::matrix<double,STATES_,1> R; //Gewichtung der Stellgroessen
    R = weight_steeringFront_, weight_steeringRear_;


    dlib::mpc<STATES_,CONTROLS_,horizon_length> controller(A,B,C,Q,R,lower,upper); //30*T ist der Zeithorizont fuer die praediktion, d.h. 30 Zeitschritte wird in die Zukunft simuliert

    dlib::matrix<double,STATES_,1> target[horizon_length];
    for(int i=0; i<horizon_length; i++){
        target[i]=y_soll[i], phi_soll[i];
        controller.set_target(target[i],i);
    }

//    one target point
//    target=y_soll, phi_soll;
//    controller.set_target(target);


    //Stellschrauben fuer performance
    //controller.set_epsilon(0.05);
    //controller.set_max_iterations(300);

    dlib::matrix<double,STATES_,1> current_state;
    current_state = 0, 0;

    dlib::matrix<double,CONTROLS_,1> action = controller(current_state); //loese MPC Problem


    steering_front = action(0,0);
    steering_rear = action(1,0);
    //TODO get angle per time
    double v_desired = msg->points[0].twist.x;
    if(std::isnan(steering_front) || std::isnan(steering_rear || std::isnan(v_desired)) ){
        ROS_ERROR_STREAM("invalid vals: " <<steering_front <<" " <<steering_rear) ;
    }

    //state.targetDistance = trajectoryPoint.position.length(); //TODO absolutwert no idea?!
    //Set values
    drive_ros_uavcan::phoenix_msgs__NucDriveCommand drive_command_msg;

    drive_command_msg.phi_f = steering_front; //publish as driving command
    drive_command_msg.phi_r = steering_rear;
    drive_command_msg.lin_vel = v_desired;
    nuc_command_pub_.publish(drive_command_msg);
    ROS_INFO_NAMED(stream_name_, "Steering front = %.5f", drive_command_msg.phi_f * 180.f / M_PI);
    ROS_INFO_NAMED(stream_name_, "Steering rear = %.5f", drive_command_msg.phi_r * 180.f / M_PI);

    return;
}