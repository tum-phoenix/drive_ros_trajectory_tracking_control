//
// Created by sebastian on 02.07.19.
//

#ifndef SRC_MODEL_PREDICTIVE_CONTROLLER_DLIB_H
#define SRC_MODEL_PREDICTIVE_CONTROLLER_DLIB_H


class ModelPredictiveController_dlib : public TrajectoryTrackingController{
public:
    ModelPredictiveController(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ModelPredictiveController();
private:
    void trajectoryCB(const drive_ros_msgs::DrivingLineConstPtr &msg);

    std::string stream_name_ = "ModelPredictiveController_dlib";

    // control parameters
    const int STATES_ = 2; //number of states (y and phi)
    const int CONTROLS_ = 2; //number of control inputs (steering_front and steering_rear)
    double T_ = 0.1;
    double weight_y_;
    double weight_phi_;
    double weight_steeringFront_;
    double weight_steeringRear_;
    float minForwardDist_=0.0; //war 1.0 in gernerator

};


#endif //SRC_MODEL_PREDICTIVE_CONTROLLER_DLIB_H
