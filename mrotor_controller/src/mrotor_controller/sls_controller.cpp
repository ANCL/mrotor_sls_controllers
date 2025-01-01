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
    sls_state_pub_ = nh_.advertise<controller_msgs::SlsState> ("mrotor_sls_controller/sls_state", 1);
    sls_force_pub_ = nh_.advertise<controller_msgs::SlsForce> ("mrotor_sls_controller/sls_force", 1);

    /* Timer */
    // cmdloop_timer_ = nh_.createTimer(ros::Duration(0.004), &mrotorSlsCtrl::cmdloopCb, this);  
    
    nh_private_.param<double>("cable_length", cable_length_, 0.85);
    nh_private_.param<double>("load_mass", load_mass_, 0.25);

    last_stage_ = ros::Time::now();

    targetPos_ << c_x_, c_y_, c_z_; 
    targetRadium_ << r_x_, r_y_, r_z_;
    targetFrequency_ << fr_x_, fr_y_, fr_z_;
    targetPhase_ << ph_x_, ph_y_, ph_z_;
}

mrotorSlsCtrl::~mrotorSlsCtrl(){
    // Destructor 
}

void mrotorSlsCtrl::gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    /* Match links on the first call*/
    if(!gazebo_link_name_matched_ && init_complete_){
        ROS_INFO_STREAM("SLS gazebo");
        ROS_INFO("[gazeboLinkStateCb] Matching Gazebo Links");
        int n_name = sizeof(gazebo_link_name_)/sizeof(*gazebo_link_name_); 
        ROS_INFO_STREAM("[gazeboLinkStateCb] n_name=" << n_name);
        int n_link = mav_num_*8+2+1; 
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
        diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
        gazebo_last_called_ = ros::Time::now();

        /* Read Gazebo Link States*/
        // Drone
        mavPos_ = toEigen(msg -> pose[drone_link_index_].position);
        mavVel_ = toEigen(msg -> twist[drone_link_index_].linear);
        mavAtt_(0) = msg -> pose[drone_link_index_].orientation.w;
        mavAtt_(1) = msg -> pose[drone_link_index_].orientation.x;
        mavAtt_(2) = msg -> pose[drone_link_index_].orientation.y;
        mavAtt_(3) = msg -> pose[drone_link_index_].orientation.z;    
        mavRate_ = toEigen(msg -> twist[drone_link_index_].angular);
        // Load 
        loadPos_ = toEigen(msg -> pose[10].position);
        loadVel_ = toEigen(msg -> twist[10].linear);
        // Pendulum
        pendAngle_ = loadPos_ - mavPos_;
        pendAngle_ = pendAngle_ / pendAngle_.norm();
        pendRate_ = pendAngle_.cross(loadVel_ - mavVel_);

        // // Finite Difference
        // if(finite_diff_enabled_){
        //     if(diff_t_ > DBL_MIN) {
        //         mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
        //         loadVel_ = (loadPos_ - loadPos_prev_) / diff_t_;
        //         pendRate_ = pendAngle_.cross((pendAngle_ - pendAngle_prev_) / diff_t_);
        //     }

        //     else {
        //         mavVel_ = mavVel_prev_;
        //         loadVel_ = loadVel_prev_;
        //         pendRate_ = pendRate_prev_;
        //     }
        // }

        
        sls_state_.header.stamp = ros::Time::now();
        sls_state_.sls_state[0] = loadPos_(1);
        sls_state_.sls_state[1] = loadPos_(0);
        sls_state_.sls_state[2] = -loadPos_(2);
        sls_state_.sls_state[3] = pendAngle_(1);
        sls_state_.sls_state[4] = pendAngle_(0);
        sls_state_.sls_state[5] = -pendAngle_(2);
        sls_state_.sls_state[6] = loadVel_(1);
        sls_state_.sls_state[7] = loadVel_(0);
        sls_state_.sls_state[8] = -loadVel_(2); 
        sls_state_.sls_state[9] = pendRate_(1);
        sls_state_.sls_state[10] = pendRate_(0);
        sls_state_.sls_state[11] = -pendRate_(2);
        sls_state_pub_.publish(sls_state_);   

        /* Publish Control Commands*/
        exeControl();

        /* Iteration */
        mavPos_prev_ = mavPos_;
        mavVel_prev_ = mavVel_;
        loadPos_prev_ = loadPos_;
        loadVel_prev_ = loadVel_;
        pendAngle_prev_ = pendAngle_;
        pendRate_prev_ = pendRate_;


    }
}

Eigen::Vector3d mrotorSlsCtrl::applyQuasiSlsCtrl(){
    double target_force_ned[3];
    double K[10] = {Kpos_x_, Kvel_x_, Kacc_x_, Kjer_x_, Kpos_y_, Kvel_y_, Kacc_y_, Kjer_y_, Kpos_z_, Kvel_z_};
    double param[4] = {load_mass_, mav_mass_, cable_length_, gravity_acc_};
    double ref[12] = {
        targetRadium_(1), targetFrequency_(1), targetPos_(1), targetPhase_(1), 
        targetRadium_(0), targetFrequency_(0), targetPos_(0), targetPhase_(0), 
        targetRadium_(2), targetFrequency_(2), -targetPos_(2), targetPhase_(2)};
    if(!traj_tracking_enabled_) {
        for(int i=0; i<12; i++){
            if((i+2)%4!=0) ref[i]=0;
        }
    }
    double sls_state_array[12];

    // for(i=0;i<12;i++){
    //     ROS_INFO_STREAM("K_i" << K[i]);
    // }
    // for(i=0;i<4;i++){
    //     ROS_INFO_STREAM("param_i" << param[i]);
    // }
    // for(i=0;i<12;i++){
    //     ROS_INFO_STREAM("ref_i" << ref[i]);
    // }

    for(int i=0; i<12;i++){
       sls_state_array[i] = sls_state_.sls_state[i];
    }
    const double t = ros::Time::now().toSec() - traj_tracking_last_called_.toSec();
    QSFGeometricController(sls_state_array, K, param, ref, t, target_force_ned);
    sls_force_.header.stamp = ros::Time::now();
    sls_force_.sls_force[0] = target_force_ned[0];
    sls_force_.sls_force[1] = target_force_ned[1];
    sls_force_.sls_force[2] = target_force_ned[2];
    sls_force_pub_.publish(sls_force_);
    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[1] / mav_mass_;
    a_des(1) = target_force_ned[0] / mav_mass_;
    a_des(2) = -target_force_ned[2] / mav_mass_;

    Eigen::Vector3d a_fb = a_des + gravity_;

    if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;

    a_des = a_fb - gravity_;
    
    return a_des;
}

void mrotorSlsCtrl::updateReference(){
    double t = ros::Time::now().toSec();
    double sp_x = c_x_ + r_x_ * std::sin(fr_x_ * t + ph_x_);
    double sp_y = c_y_ + r_y_ * std::sin(fr_y_ * t + ph_y_);
    double sp_z = c_z_ + r_z_ * std::sin(fr_z_ * t + ph_z_);
    double sp_x_dt = r_x_ * fr_x_ * std::cos(fr_x_ * t + ph_x_);
    double sp_y_dt = r_y_ * fr_y_ * std::cos(fr_y_ * t + ph_y_);
    double sp_z_dt = r_z_ * fr_z_ * std::cos(fr_z_ * t + ph_z_);

    if(mission_enabled_){
        switch(mission_stage_){
            case 0:
                if(!mission_initialized_){
                    ROS_INFO("[exeMission] Mission started at case 0");
                    mission_initialized_ = true;
                }
                targetPos_ << c_x_, c_y_, c_z_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 1:
                targetPos_ << c_x_1_, c_y_1_, c_z_1_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 2:
                targetPos_ << c_x_2_, c_y_2_, c_z_2_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 3:
                targetPos_ << c_x_3_, c_y_3_, c_z_3_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 4:
                targetPos_ << c_x_, c_y_, c_z_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                checkMissionStage(10);
                break;
            case 5:
                targetPos_ << c_x_, c_y_, c_z_; 
                targetRadium_ << r_x_, r_y_, r_z_;
                targetFrequency_ << fr_x_, fr_y_, fr_z_;
                targetPhase_ << ph_x_, ph_y_, ph_z_;
                traj_tracking_enabled_ = true;
                checkMissionStage(20);
                break;
            default:
                targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_; 
                targetRadium_ << 0, 0, 0;
                targetFrequency_ << 0, 0, 0;
                targetPhase_ << 0, 0, 0;
                traj_tracking_enabled_ = false;
                if(ros::Time::now().toSec() - mission_last_called_.toSec() >= 10){
                    ROS_INFO("[exeMission] Mission Accomplished");
                    mission_last_called_ = ros::Time::now();
                }
        }
    }

    else {
        mission_last_called_ = ros::Time::now();
        if(mission_stage_ == 5) {
            traj_tracking_enabled_ = false;
        }
        mission_stage_ = 0;

        targetPos_ << c_x_, c_y_, c_z_; 
        targetRadium_ << r_x_, r_y_, r_z_;
        targetFrequency_ << fr_x_, fr_y_, fr_z_;
        targetPhase_ << ph_x_, ph_y_, ph_z_;


        // // Entering tracking
        // if(!last_tracking_state_ && traj_tracking_enabled_ && (mav_num_>1 && (std::abs(mavPos_(0)-sp_x)>0.1 || std::abs(mavPos_(1)-sp_y)>0.1))) {
        //     targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
        //     targetVel_ << 0.0, 0.0, 0.0;
        //     targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero  
        //     // ROS_INFO("1");
        // }    

        // // Running tracking
        // else if(last_tracking_state_ && traj_tracking_enabled_) {
        //     targetPos_ << sp_x, sp_y, sp_z;
        //     targetVel_ << sp_x_dt, sp_y_dt, sp_z_dt;
        //     last_tracking_state_ = traj_tracking_enabled_;
        //     // ROS_INFO("2");
        // }

        // // Exiting from tracking
        // else if(last_tracking_state_ && (mav_num_>1 && (std::abs(mavPos_(0)-pos_x_0_)>tracking_exit_min_error_ || std::abs(mavPos_(1)-pos_y_0_)>tracking_exit_min_error_))) {
        //     targetPos_ << sp_x, sp_y, sp_z;
        //     targetVel_ << sp_x_dt, sp_y_dt, sp_z_dt;   
        //     // ROS_INFO("3");     
        // }

        // // Initial Point
        // else {
        //     targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
        //     targetVel_ << 0.0, 0.0, 0.0;
        //     targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero  
        //     last_tracking_state_ = traj_tracking_enabled_;
        //     // ROS_INFO("4");
        // }
    }
}

void mrotorSlsCtrl::checkMissionStage(double mission_time_span) {
    if(ros::Time::now().toSec() - mission_last_called_.toSec() >= mission_time_span) {
        mission_last_called_ = ros::Time::now();
        mission_stage_ += 1;
        ROS_INFO_STREAM("[exeMission] Stage " << mission_stage_-1 << " ended, switching to stage " << mission_stage_);
    }
}

void mrotorSlsCtrl::exeControl(void){
    // printf("SLS Control EXE\n");
    if(init_complete_){
        if(traj_tracking_enabled_ && !traj_tracking_enabled_last_) {
            traj_tracking_last_called_ = ros::Time::now();
        }
        traj_tracking_enabled_last_ = traj_tracking_enabled_;

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

// void mrotorSlsCtrl::cmdloopCb(const ros::TimerEvent &event) {
//     /* Publish Control Commands*/
//     mrotorSlsCtrl::exeControl();
// }