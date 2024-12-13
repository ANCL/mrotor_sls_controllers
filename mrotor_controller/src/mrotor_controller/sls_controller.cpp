#include "mrotor_controller/sls_controller.hpp"
#include "mrotor_controller/QSFGeometricController.h"

/*================================================================= SLS Controller =================================================================*/

mrotorSlsCtrl::mrotorSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): mrotorCtrl(nh, nh_private) {
    // ROS_INFO_STREAM(gravity_acc_);

    /* Subscribers */
    gazebo_link_state_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &mrotorSlsCtrl::gazeboLinkStateCb, this, ros::TransportHints().tcpNoDelay());
    switch(mav_id_) {
        case 1:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_1/px4vision_1", 1000, &mrotorSlsCtrl::viconDrone1Cb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        case 2:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_2/px4vision_2", 1000, &mrotorSlsCtrl::viconDrone2Cb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        default: 
            break;
    }
    vicon_load_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/load_1/load_1", 1000, &mrotorSlsCtrl::viconLoad1Cb, this, ros::TransportHints().tcpNoDelay()); 

    /* Publishers */
    sls_state_pub_ = nh_.advertise<controller_msgs::SlsState> ("mrotor_sls_controller/sls_state", 10);
    
    nh_private_.param<double>("cable_length", cable_length_, 0.85);
    nh_private_.param<double>("load_mass", load_mass_, 0.25);

    last_stage_ = ros::Time::now();
}

mrotorSlsCtrl::~mrotorSlsCtrl(){
    // Destructor 
}

void mrotorSlsCtrl::gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    /* Match links on the first call*/
    if(!gazebo_link_name_matched_ && init_complete_){
        ROS_INFO("[gazeboLinkStateCb] Matching Gazebo Links");
        int n_name = sizeof(gazebo_link_name_)/sizeof(*gazebo_link_name_); 
        ROS_INFO_STREAM("[gazeboLinkStateCb] n_name=" << n_name);
        int n_link = mav_num_*8+2+2; 
        ROS_INFO_STREAM("[gazeboLinkStateCb] n_link=" << n_link);
        int temp_index[n_name];
        for(int i=0; i<n_link; i++){
            for(int j=0; j<n_name; j++){
                if(msg->name[i] == gazebo_link_name_[j]){
                    temp_index[j] = i;
                    
                };
            }
        }
        drone_link_index_ = temp_index[mav_id_-1];
        gazebo_link_name_matched_ = true; ROS_INFO_STREAM("drone_link_index_=" << drone_link_index_);
        ROS_INFO("[gazeboLinkStateCb] Matching Complete");
    }

    if(gazebo_link_name_matched_) {

    // ROS_INFO_STREAM("SLS gazebo");
    diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
    gazebo_last_called_ = ros::Time::now();
    /* Read Gazebo Link States*/
    // Drone
    mavPos_ = toEigen(msg -> pose[2].position);
    mavVel_ = toEigen(msg -> twist[2].linear);
    mavAtt_(0) = msg -> pose[2].orientation.w;
    mavAtt_(1) = msg -> pose[2].orientation.x;
    mavAtt_(2) = msg -> pose[2].orientation.y;
    mavAtt_(3) = msg -> pose[2].orientation.z;    
    mavRate_ = toEigen(msg -> twist[2].angular);
    // Load 
    loadPos_ = toEigen(msg -> pose[10].position);
    loadVel_ = toEigen(msg -> twist[10].linear);

    pendAngle_ = (loadPos_ - mavPos_);
    pendRate_ = loadVel_ - mavVel_;

    sls_state_.header.stamp = ros::Time::now();
    sls_state_.sls_state[0] = loadPos_(0);
    sls_state_.sls_state[1] = loadPos_(1);
    sls_state_.sls_state[2] = loadPos_(2);
    sls_state_.sls_state[3] = pendAngle_(0);
    sls_state_.sls_state[4] = pendAngle_(1);
    sls_state_.sls_state[5] = pendAngle_(2);
    sls_state_.sls_state[6] = loadVel_(0);
    sls_state_.sls_state[7] = loadVel_(1);
    sls_state_.sls_state[8] = loadVel_(2); 
    sls_state_.sls_state[9] = pendRate_(0);
    sls_state_.sls_state[10] = pendRate_(1);
    sls_state_.sls_state[11] = pendRate_(2);
    sls_state_pub_.publish(sls_state_);   

    /* Publish Control Commands*/
    exeControl();

    }
}

Eigen::Vector3d mrotorSlsCtrl::applyQuasiSlsCtrl(){
    double target_force_ned[3];
    double K[10] = {Kpos_x_, Kvel_x_, Kacc_x_, Kjer_x_, Kpos_y_, Kvel_y_, Kacc_y_, Kacc_z_, Kpos_z_, Kvel_z_};
    double param[4] = {load_mass_, mav_mass_, cable_length_, gravity_acc_};
    double ref[12] = {r_x_, c_x_, fr_x_, ph_x_, r_y_, c_y_, fr_y_, ph_y_, r_z_, c_z_, fr_z_, ph_z_,};
    double sls_state_array[12];
    for(int i=0; i<12;i++){
       sls_state_array[i] = sls_state_.sls_state[i];
    }
    const double t = ros::Time::now().toSec() - last_stage_.toSec();
    QSFGeometricController(sls_state_array, K, param, ref, t, target_force_ned);
    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[0] / mav_mass_;
    a_des(1) = -target_force_ned[1] / mav_mass_;
    a_des(2) = -target_force_ned[2] / mav_mass_;
    
    return a_des;
}

void mrotorSlsCtrl::exeControl(void){
    if(init_complete_){
        Eigen::Vector3d desired_acc;
        desired_acc = applyQuasiSlsCtrl();
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        if(ctrl_enabled_){
            pubRateCommands(cmdBodyRate_, q_des);
        }
        else{
            pubTargetPose(pos_x_0_, pos_y_0_, pos_z_0_);
            debugRateCommands(cmdBodyRate_, q_des);
        }
        updateReference();
    }
}

void mrotorSlsCtrl::viconDrone1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg){

}

void mrotorSlsCtrl::viconDrone2Cb(const geometry_msgs::TransformStamped::ConstPtr& msg){

}

void mrotorSlsCtrl::viconLoad1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg){

}