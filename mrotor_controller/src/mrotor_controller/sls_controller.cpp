#include "mrotor_controller/sls_controller.hpp"
#include "mrotor_controller/QSFGeometricController.h"

/*================================================================= SLS Controller =================================================================*/

#define FD_EPSILON DBL_MIN

mrotorSlsCtrl::mrotorSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): mrotorCtrl(nh, nh_private) {
    // ROS_INFO_STREAM(gravity_acc_);

    /* Subscribers */
    gazebo_link_state_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &mrotorSlsCtrl::gazeboLinkStateCb, this, ros::TransportHints().tcpNoDelay());
    switch(mav_id_) {
        case 1:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_1/px4vision_1", 1000, &mrotorSlsCtrl::viconDrone1PoseCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        case 2:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_2/px4vision_2", 1000, &mrotorSlsCtrl::viconDrone2PoseCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        default: 
            break;
    }
    vicon_load_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/load_1/load_1", 1000, &mrotorSlsCtrl::viconLoad1PoseCb, this, ros::TransportHints().tcpNoDelay()); 

    /* Publishers */
    sls_state_raw_pub_ = nh_.advertise<controller_msgs::SlsState> ("mrotor_sls_controller/sls_state_raw", 1);
    sls_state_pub_ = nh_.advertise<controller_msgs::SlsState> ("mrotor_sls_controller/sls_state", 1);
    // sls_state_ekf_pub_ = nh_.advertise<controller_msgs::SlsState> ("mrotor_sls_controller/sls_state_ekf", 1);
    sls_force_pub_ = nh_.advertise<controller_msgs::SlsForce> ("mrotor_sls_controller/sls_force", 1);

    /* Timer */
    // cmdloop_timer_ = nh_.createTimer(ros::Duration(0.004), &mrotorSlsCtrl::cmdloopCb, this);  
    
    nh_private_.param<double>("cable_length", cable_length_, 0.85);
    nh_private_.param<double>("load_mass", load_mass_, 0.25);

    last_stage_ = ros::Time::now();
    vicon_load_last_called_ = ros::Time::now();

    targetPos_ << c_x_, c_y_, c_z_; 
    targetRadium_ << r_x_, r_y_, r_z_;
    targetFrequency_ << fr_x_, fr_y_, fr_z_;
    targetPhase_ << ph_x_, ph_y_, ph_z_;

    P_ = Eigen::MatrixXd::Zero(12, 12);
    Q_ = Eigen::MatrixXd::Zero(12, 12);
    R_ = Eigen::MatrixXd::Zero(12, 12);

    // ekf_ = std::make_shared<mrotorSlsEKF>(mav_mass_, load_mass_, cable_length_, P_, Q_, R_, false);

    // load_vel_tau_up_ = 0.1592;
    // load_vel_tau_down_ = 0.1592;
    // load_vel_filter_ = new FirstOrderFilter(load_vel_tau_up_, load_vel_tau_down_, loadVel_);

    double load_vel_cutoff_freq;
    double load_vel_q;
    bool load_vel_verbose;
    double pend_rate_cutoff_freq;
    double pend_rate_q;
    bool pend_rate_verbose;
    nh_private_.param<double>("load_vel_cutoff_freq", load_vel_cutoff_freq, 30);
    nh_private_.param<double>("load_vel_q", load_vel_q, 0.625);
    nh_private_.param<bool>("load_vel_verbose", load_vel_verbose, false);
    nh_private_.param<double>("pend_rate_cutoff_freq", pend_rate_cutoff_freq, 30);
    nh_private_.param<double>("pend_rate_q", pend_rate_q, 0.625);
    nh_private_.param<bool>("pend_rate_verbose", pend_rate_verbose, false);


    load_vel_filter_.reset(new SecondOrderFilter<Eigen::Vector3d>(load_vel_cutoff_freq, load_vel_q, load_vel_verbose));
    pend_rate_filter_.reset(new SecondOrderFilter<Eigen::Vector3d>(load_vel_cutoff_freq, pend_rate_q, pend_rate_verbose));
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
        /* Get Gazebo Link States*/
        // >>> Pose
        // Drone
        mavPos_ = toEigen(msg -> pose[drone_link_index_].position);
        mavAtt_(0) = msg -> pose[drone_link_index_].orientation.w;
        mavAtt_(1) = msg -> pose[drone_link_index_].orientation.x;
        mavAtt_(2) = msg -> pose[drone_link_index_].orientation.y;
        mavAtt_(3) = msg -> pose[drone_link_index_].orientation.z;    
        // Load 
        loadPos_ = toEigen(msg -> pose[10].position);  
        // Pendulum
        pendAngle_ = loadPos_ - mavPos_;
        pendAngle_ = pendAngle_ / pendAngle_.norm();

        // >>> Velocity
        mavVel_ = toEigen(msg -> twist[drone_link_index_].linear);
        mavRate_ = toEigen(msg -> twist[drone_link_index_].angular);
        loadVel_ = toEigen(msg -> twist[10].linear);
        pendRate_ = pendAngle_.cross(loadVel_ - mavVel_);

        if(!cmdloop_enabled_) {
            diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
            gazebo_last_called_ = ros::Time::now();

            if(finite_diff_enabled_){
                applyFiniteDiffSys();
            }

            applyLowPassFilter();

            loadSlsState();  
            sls_state_pub_.publish(sls_state_); 

            // // EKF
            // if(ctrl_enabled_) {
            //     controller_msgs::SlsState sls_state_ekf;
            //     if(!ekf_init_) {
            //         ekf_ -> resetInitState(sls_state_);
            //         ekf_init_ = true;
            //     }    

            //     if(diff_t_ > FD_EPSILON) {
            //         sls_state_ekf = ekf_ -> updateFilter(sls_state_, sls_force_, diff_t_);
            //         sls_state_ekf_pub_.publish(sls_state_ekf); 
            //     }

            //     if(ekf_enabled_) {   
            //         sls_state_ = sls_state_ekf;
            //     }

            //     else {
            //         sls_state_pub_.publish(sls_state_); 
            //     }
            // }

            // else {
            //     ekf_init_ = false;
            // }
            

            exeControl(); 

            // ROS_INFO_STREAM("loadpose_err: " << loadPos_(2) - 1.0);
        }
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

    for(int i=0; i<12;i++){
       sls_state_array[i] = sls_state_.sls_state[i];
    }
    const double t = ros::Time::now().toSec() - traj_tracking_last_called_.toSec();
    QSFGeometricController(sls_state_array, K, param, ref, t, target_force_ned);

    // if(std::abs(target_force_ned[0]) > 2 ) {
    //     target_force_ned[0] = std::copysign(2, target_force_ned[0]);
    // }

    // if(std::abs(target_force_ned[1]) > 2 ) {
    //     target_force_ned[1] = std::copysign(2,target_force_ned[1]);
    // }

    sls_force_.header.stamp = ros::Time::now();
    sls_force_.sls_force[0] = target_force_ned[0];
    sls_force_.sls_force[1] = target_force_ned[1];
    sls_force_.sls_force[2] = target_force_ned[2];
    sls_force_pub_.publish(sls_force_);
    // ROS_INFO_STREAM("SLS Force: " << target_force_ned[2]);
    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[1] / mav_mass_;
    a_des(1) = target_force_ned[0] / mav_mass_;
    a_des(2) = -target_force_ned[2] / mav_mass_;

    Eigen::Vector3d a_fb = a_des + gravity_;

    if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;

    // rotor drag compensation
    Eigen::Vector3d a_rd;
    if(drag_comp_enabled_) {
        a_rd = compensateRotorDrag(t);
    }
    else {
        a_rd = Eigen::Vector3d::Zero();
    } 

    a_des = a_fb - a_rd - gravity_;
    
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
                    mission_initialized_ = false;
                }
        }
    }

    else {
        mission_last_called_ = ros::Time::now();
        if(mission_stage_ == 5) {
            traj_tracking_enabled_ = false;
        }
        mission_stage_ = 0;
        mission_initialized_ = false;
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
    ROS_DEBUG("SLS Control EXE");
    if(init_complete_){
        if(traj_tracking_enabled_ && !traj_tracking_enabled_last_) {
            traj_tracking_last_called_ = ros::Time::now();
        }
        traj_tracking_enabled_last_ = traj_tracking_enabled_;

        Eigen::Vector3d desired_acc;
        desired_acc = applyQuasiSlsCtrl();
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        if(ctrl_enabled_){
            pubRateCommands(cmdBodyRate_, q_des_);
        }
        else{
            pubTargetPose(pos_x_0_, pos_y_0_, pos_z_0_);
            debugRateCommands(cmdBodyRate_, q_des_);
        }
        updateReference();
    }
}




void mrotorSlsCtrl::cmdloopCb(const ros::TimerEvent &event) {
    if(cmdloop_enabled_) {
        diff_t_ = ros::Time::now().toSec() - cmdloop_last_called_.toSec(); 
        cmdloop_last_called_ = ros::Time::now();

        if(finite_diff_enabled_){
            applyFiniteDiffSys();
        }

        loadSlsState();  

        exeControl(); 
    }
}

Eigen::Vector3d mrotorSlsCtrl::transformPose(Eigen::Vector3d oldPose, Eigen::Vector3d offsetVector) {
    Eigen::Vector3d newPose;
    Eigen::Vector4d oldPose4, newPose4;
    Eigen::Matrix4d transMatrix;

    // >>> Translation Matrix
    transMatrix <<  1, 0, 0, offsetVector(0),
                    0, 1, 0, offsetVector(1),
                    0, 0, 1, offsetVector(2), 
                    0, 0, 0, 1;
    // std::cout << transMatrix << std::endl;

    // >>> old pose
    oldPose4.head(3) = oldPose;
    oldPose4(3) = 1;
    // std::cout << oldPose4 << std::endl;

    // >>> new pose
    newPose4 = transMatrix*oldPose4;
    newPose = newPose4.head(3);
    // std::cout << newPose << std::endl;

    return newPose;
}

Eigen::Vector3d mrotorSlsCtrl::compensateRotorDrag(double t) {
    bool use_feedback = true;
    // mav vel ref
    Eigen::Vector3d loadVelRDC, loadAccRDC, loadVelFF, loadAccFF, loadVelFB, loadAccFB;
    loadVelFF(0) = targetRadium_(0) * targetFrequency_(0) * std::cos(targetFrequency_(0) * t + targetPhase_(0));
    loadVelFF(1) = targetRadium_(1) * targetFrequency_(1) * std::cos(targetFrequency_(1) * t + targetPhase_(1));
    loadVelFF(2) = targetRadium_(2) * targetFrequency_(2) * std::cos(targetFrequency_(2) * t + targetPhase_(2));
    loadAccFF(0) = -targetRadium_(0) * std::pow(targetFrequency_(0), 2) * std::sin(targetFrequency_(0) * t + targetPhase_(0));
    loadAccFF(1) = -targetRadium_(1) * std::pow(targetFrequency_(1), 2) * std::sin(targetFrequency_(1) * t + targetPhase_(1));
    loadAccFF(2) = -targetRadium_(2) * std::pow(targetFrequency_(2), 2) * std::sin(targetFrequency_(2) * t + targetPhase_(2));
    loadVelFB = loadVel_;
    loadAccFB = loadAcc_;

    if(traj_tracking_enabled_) {
        loadVelRDC = loadVelFF;
        loadAccRDC = loadAccFF;
    }
    else if(use_feedback) {
        loadVelRDC = loadVelFB;
        loadAccRDC = loadAccFB;
    }
    else {
        loadVelRDC = Eigen::Vector3d::Zero();
        loadAccRDC = Eigen::Vector3d::Zero();
    }
    
    const Eigen::Vector3d mavVelRef = transformPose(loadVelRDC, -pendRate_);
    const Eigen::Vector3d mavAccRef = transformPose(loadAccRDC, -pendAngularAcc_);

    // mav RotMat ref
    const Eigen::Vector4d q_ref = acc2quaternion(mavAccRef - gravity_, mavYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    // drag coefficient
    rotorDragD_ << rotorDragD_x_, rotorDragD_y_, rotorDragD_z_;

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = -R_ref * rotorDragD_.asDiagonal() * R_ref.transpose() * mavVelRef;  // Rotor drag


    // ROS_INFO_STREAM(a_rd);

    return a_rd;
}


void mrotorSlsCtrl::loadSlsState() {
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
}

void mrotorSlsCtrl::loadSlsStateRaw() {
    sls_state_raw_.header.stamp = ros::Time::now();
    sls_state_raw_.sls_state[0] = loadPos_(1);
    sls_state_raw_.sls_state[1] = loadPos_(0);
    sls_state_raw_.sls_state[2] = -loadPos_(2);
    sls_state_raw_.sls_state[3] = pendAngle_(1);
    sls_state_raw_.sls_state[4] = pendAngle_(0);
    sls_state_raw_.sls_state[5] = -pendAngle_(2);
    sls_state_raw_.sls_state[6] = loadVel_(1);
    sls_state_raw_.sls_state[7] = loadVel_(0);
    sls_state_raw_.sls_state[8] = -loadVel_(2); 
    sls_state_raw_.sls_state[9] = pendRate_(1);
    sls_state_raw_.sls_state[10] = pendRate_(0);
    sls_state_raw_.sls_state[11] = -pendRate_(2);
}


void mrotorSlsCtrl::applyIteration(void) {
    /* Iteration */
    mavPos_prev_ = mavPos_;
    mavVel_prev_ = mavVel_;
    mavAtt_prev_ = mavAtt_;
    mavRate_prev_ = mavRate_;
    loadPos_prev_ = loadPos_;
    loadVel_prev_ = loadVel_;
    loadAcc_prev_ = loadAcc_;
    pendAngle_prev_ = pendAngle_;
    pendRate_prev_ = pendRate_;
    pendAngularAcc_prev_ = pendAngularAcc_;
}

void mrotorSlsCtrl::applyFiniteDiffSys(void) {
    // ROS_INFO_STREAM("diff_t: " << diff_t_);
    // ROS_INFO_STREAM("cmdloop: " << cmdloop_enabled_);
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * mavAtt_prev_;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, mavAtt_);

    if(diff_t_ > FD_EPSILON) {
        mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
        loadVel_ = (loadPos_ - loadPos_prev_) / diff_t_;
        loadAcc_ = (loadVel_ - loadVel_prev_) / diff_t_;
        mavRate_(0) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(1);
        mavRate_(1) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(2);
        mavRate_(2) = (2.0 / diff_t_) * std::copysign(1.0, qe(0)) * qe(3);
        pendRate_ = pendAngle_.cross((pendAngle_ - pendAngle_prev_) / diff_t_);
        pendAngularAcc_ = (pendRate_ - pendRate_prev_) / diff_t_;

        applyIteration();
    }

    else {
        // printf("diff_t_: %.8f", diff_t_);
        // ROS_INFO_STREAM(mavPos_ - mavPos_prev_); 
        // ROS_INFO_STREAM(loadPos_ - loadPos_prev_);
        mavVel_ = mavVel_prev_;
        mavRate_ = mavRate_prev_;
        loadVel_ = loadVel_prev_;
        loadAcc_ = loadAcc_prev_;
        pendRate_ = pendRate_prev_;
        pendAngularAcc_ = pendAngularAcc_prev_;
    }

    // if(diff_t_ != 0.004) ROS_INFO_STREAM("FD: Time interval varied!" << mavVel_prev_ << "" << mavVel_);
}


void mrotorSlsCtrl::applyLowPassFilter(void) {
    if(lpf_enabled_) {
        loadVel_ = load_vel_filter_ -> updateFilter(loadVel_, diff_t_);
        pendRate_ = pend_rate_filter_ -> updateFilter(pendRate_, diff_t_);
    }
}

void mrotorSlsCtrl::viconDrone1PoseCb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("Vicon Drone1 Cb");
    readViconDronePose(msg);
    exeControl();
}

void mrotorSlsCtrl::viconDrone2PoseCb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // ROS_INFO_STREAM("Vicon Drone2 Cb");
    readViconDronePose(msg);
    exeControl();
}

void mrotorSlsCtrl::viconLoad1PoseCb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    // ROS_INFO_STREAM("Vicon Load1 Cb");
    readViconLoadPose(msg);
}


void mrotorSlsCtrl::readViconDronePose(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    
    diff_t_ = ros::Time::now().toSec() - vicon_drone_last_called_.toSec(); 
    vicon_drone_last_called_ = ros::Time::now();
    // ROS_INFO_STREAM("diff_t_: " << diff_t_);

    mavPos_ = toEigen(msg -> transform.translation);
    mavAtt_(0) = msg -> transform.rotation.w;
    mavAtt_(1) = msg -> transform.rotation.x;
    mavAtt_(2) = msg -> transform.rotation.y;
    mavAtt_(3) = msg -> transform.rotation.z;  

    // >>> Debug
    // loadPos_ = mavPos_;
    // loadPos_(2) += -0.85;

    pendAngle_ = loadPos_ - mavPos_;
    pendAngle_ = pendAngle_ / pendAngle_.norm();

    applyFiniteDiffSys();
    loadSlsStateRaw();
    sls_state_raw_pub_.publish(sls_state_raw_); 
    applyLowPassFilter();
    loadSlsState();
    sls_state_pub_.publish(sls_state_); 
    exeControl(); 

}

void mrotorSlsCtrl::readViconLoadPose(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    loadPos_ = toEigen(msg -> transform.translation);
}

