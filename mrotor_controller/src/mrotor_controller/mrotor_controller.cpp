#include "mrotor_controller/mrotor_controller.hpp"
#include "mrotor_controller/nonlinear_attitude_control.h"

mrotorCtrl::mrotorCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): nh_(nh), nh_private_(nh_private) {
    /* Retrieve vehicle total numbers & id */
    nh_private_.param<int>("mav_num", mav_num_, 1);
    nh_private_.param<int>("mav_id", mav_id_, 1);
    
    /* Subscribers */
    mav_state_sub_ = nh_.subscribe<mavros_msgs::State> ("mavros/state", 10, &mrotorCtrl::mavstateCb, this, ros::TransportHints().tcpNoDelay()); 
    gazebo_link_state_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &mrotorCtrl::gazeboLinkStateCb, this, ros::TransportHints().tcpNoDelay());
    switch(mav_id_) {
        case 1:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_1/px4vision_1", 1000, &mrotorCtrl::viconCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        case 2:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_2/px4vision_2", 1000, &mrotorCtrl::viconCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        default: 
            break;
    }

    /* Publishers */
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    target_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("mavros/setpoint_raw/attitude", 10);
    target_attitude_debug_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("mrotor_controller/setpoint_raw/attitude", 10);
    system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);


    /* Service Clients */
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    /* Timer */

    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &mrotorCtrl::statusloopCb, this);

    /* Offboard Rate */
    ros::Rate rate(20.0);

    /* Variables */
    double attctrl_tau;
    
    /* Retrieve Parameters */
    // booleans
    nh_private_.param<bool>("ctrl_enabled", ctrl_enabled_, false);
    nh_private_.param<bool>("rate_ctrl_enabled", rate_ctrl_enabled_, true);
    nh_private_.param<bool>("sim_enabled", sim_enabled_, true);
    nh_private_.param<bool>("finite_diff_enabled", finite_diff_enabled_, true);
    // drone physical constants
    nh_private_.param<double>("mav_mass", mav_mass_, 1.56);    
    nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
    // drone Yaw
    nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
    // attitude controller
    nh_private_.param<double>("attctrl_tau", attctrl_tau, 0.3);
    // throttle normalization
    nh_private_.param<double>("max_thrust_force", max_thrust_force_, 31.894746920044025);
    nh_private_.param<double>("normalized_thrust_constant", norm_thrust_const_, 0.05055);
    nh_private_.param<double>("normalized_thrust_offset",norm_thrust_offset_, 0.0); // -0.0335
    // Controller Gains
    nh_private_.param<double>("Kp_x", Kpos_x_, 10.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 10.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 20.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 5.0);
    nh_private_.param<double>("Kv_y", Kvel_y_, 5.0);
    nh_private_.param<double>("Kv_z", Kvel_z_, 10.0);
    nh_private_.param<double>("Ka_x", Kacc_x_, 0.0);
    nh_private_.param<double>("Ka_y", Kacc_y_, 0.0);
    nh_private_.param<double>("Ka_z", Kacc_z_, 0.0);
    nh_private_.param<double>("Kj_x", Kjer_x_, 0.0);
    nh_private_.param<double>("Kj_y", Kjer_y_, 0.0);
    nh_private_.param<double>("Kj_z", Kjer_z_, 0.0);     
    // Reference
    nh_private_.param<double>("c_x", c_x_, 0.0);
    nh_private_.param<double>("c_y", c_y_, 0.0);
    nh_private_.param<double>("c_z", c_z_, 1.0);    
    nh_private_.param<double>("r_x", r_x_, 0.0);
    nh_private_.param<double>("r_y", r_y_, 0.0);    
    nh_private_.param<double>("r_z", r_z_, 0.0);  
    nh_private_.param<double>("fr_x", fr_x_, 0.0);
    nh_private_.param<double>("fr_y", fr_y_, 0.0);    
    nh_private_.param<double>("fr_z", fr_z_, 0.0);      
    nh_private_.param<double>("ph_x", ph_x_, 0.0);
    nh_private_.param<double>("ph_y", ph_y_, 0.0);    
    nh_private_.param<double>("ph_z", ph_z_, 0.0);     
    // Initial Positions
    nh_private_.param<double>("pos_x_0", pos_x_0_, 0.0);
    nh_private_.param<double>("pos_y_0", pos_y_0_, 0.0);
    nh_private_.param<double>("pos_z_0", pos_z_0_, 1.0);     
    // tolerance
    nh_private_.param<double>("tracking_exit_min_error", tracking_exit_min_error_, 0.5); 

    /* Send some set-points before starting*/
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     pubTargetPose(0, 0, 1);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    /* Initialize Vectors */
    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;
    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;   
    targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero

    controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    ROS_INFO_STREAM(controller_);

    /* Initialization Successful Message */
    ROS_INFO_STREAM("[mrotorCtrl] Initialization Complete");
    init_complete_ = true;
    gazebo_last_called_ = ros::Time::now();
    vicon_last_called_ = ros::Time::now();
}


mrotorCtrl::~mrotorCtrl(){
    // Destructor 
}


void mrotorCtrl::mavstateCb(const mavros_msgs::State::ConstPtr& msg){
    mav_state_ = *msg;
}


void mrotorCtrl::gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    // ROS_INFO_STREAM("mav gazebo");
    
    /* Match links on the first call*/
    if(!gazebo_link_name_matched_ && init_complete_){
        ROS_INFO("[gazeboLinkStateCb] Matching Gazebo Links");
        int n_name = sizeof(gazebo_link_name_)/sizeof(*gazebo_link_name_); 
        ROS_INFO_STREAM("[gazeboLinkStateCb] n_name=" << n_name);
        int n_link = mav_num_*8+2; 
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

        /* Read Gazebo Link States*/
        if(!finite_diff_enabled_) {
            mavPos_ = toEigen(msg -> pose[drone_link_index_].position);
            mavVel_ = toEigen(msg -> twist[drone_link_index_].linear);
        }

        

        else {
            diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
            gazebo_last_called_ = ros::Time::now();
            mavPos_ = toEigen(msg -> pose[drone_link_index_].position);
            if(diff_t_ > 0) {
                mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
            }
            mavPos_prev_ = mavPos_;
            
        }

        mavAtt_(0) = msg -> pose[drone_link_index_].orientation.w;
        mavAtt_(1) = msg -> pose[drone_link_index_].orientation.x;
        mavAtt_(2) = msg -> pose[drone_link_index_].orientation.y;
        mavAtt_(3) = msg -> pose[drone_link_index_].orientation.z;    
        // mavRate_ = toEigen(msg -> twist[drone_link_index_].angular);
        
        /* Publish Control Commands*/
        exeControl();
    }
}

void mrotorCtrl::viconCb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // ROS_INFO_STREAM("Vicon Mrotor Cb");
    diff_t_ = ros::Time::now().toSec() - vicon_last_called_.toSec(); 
    vicon_last_called_ = ros::Time::now();
    mavPos_ = toEigen(msg -> transform.translation);
    if(diff_t_ > 0) {
        mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
    }
    mavPos_prev_ = mavPos_;
    mavAtt_(0) = msg -> transform.rotation.w;
    mavAtt_(1) = msg -> transform.rotation.x;
    mavAtt_(2) = msg -> transform.rotation.y;
    mavAtt_(3) = msg -> transform.rotation.z;    
    
    /* Publish Control Commands*/
    exeControl();
}

void mrotorCtrl::dynamicReconfigureCb(mrotor_controller::MrotorControllerConfig &config, uint32_t level) {
    /* Switches */
    if(ctrl_enabled_ != config.ctrl_enabled) {
        ctrl_enabled_ = config.ctrl_enabled;
        ROS_INFO("Reconfigure request : ctrl_enabled = %s ", ctrl_enabled_ ? "true" : "false");
    } 

    else if(rate_ctrl_enabled_ != config.rate_ctrl_enabled) {
        rate_ctrl_enabled_ = config.rate_ctrl_enabled;
        ROS_INFO("Reconfigure request : rate_ctrl_enabled = %s ", rate_ctrl_enabled_ ? "true" : "false");
    } 

    else if(traj_tracking_enabled_ != config.traj_tracking_enabled) {
        traj_tracking_enabled_ = config.traj_tracking_enabled;
        ROS_INFO("Reconfigure request : traj_tracking_enabled = %s ", traj_tracking_enabled_ ? "true" : "false");
    } 

    else if(finite_diff_enabled_ != config.finite_diff_enabled) {
        finite_diff_enabled_ = config.finite_diff_enabled;
        ROS_INFO("Reconfigure request : finite_diff_enabled = %s ", finite_diff_enabled_ ? "true" : "false");
    } 

    /* Max Acceleration*/
    else if (max_fb_acc_ != config.max_acc) {
        max_fb_acc_ = config.max_acc;
        ROS_INFO("Reconfigure request : max_acc = %.3f ", config.max_acc);
    }

    // // Causes large delay!
    // /* Attitude Control */
    // else if (attctrl_tau_ != config.attctrl_tau) {
    //     attctrl_tau_ = config.attctrl_tau;
    //     ROS_INFO("Reconfigure request : attctrl_tau = %.3f ", config.attctrl_tau);
    // } 

    /* Thrust */
    else if(norm_thrust_const_ != config.normalized_thrust_constant) {
        norm_thrust_const_ = config.normalized_thrust_constant;
        ROS_INFO("Reconfigure request : normalized_thrust_constant = %.3f ", norm_thrust_const_);
    }
    else if(norm_thrust_offset_ != config.normalized_thrust_offset) {
        norm_thrust_offset_ = config.normalized_thrust_offset;
        ROS_INFO("Reconfigure request : normalized_thrust_offset = %.3f ", norm_thrust_offset_);
    }

    /* Gains */
    // Position
    else if(Kpos_x_ != config.Kp_x) {
        Kpos_x_ = config.Kp_x;
        ROS_INFO("Reconfigure request : Kp_x = %.3f ", Kpos_x_);
    }
    else if(Kpos_y_ != config.Kp_y) {
        Kpos_y_ = config.Kp_y;
        ROS_INFO("Reconfigure request : Kp_y = %.3f ", Kpos_y_);
    }
    else if(Kpos_z_ != config.Kp_z) {
        Kpos_z_ = config.Kp_z;
        ROS_INFO("Reconfigure request : Kp_z = %.3f ", Kpos_z_);
    }
    // Velocity
    else if(Kvel_x_ != config.Kv_x) {
        Kvel_x_ = config.Kv_x;
        ROS_INFO("Reconfigure request : Kv_x = %.3f ", Kvel_x_);
    }
    else if(Kvel_y_ != config.Kv_y) {
        Kvel_y_ = config.Kv_y;
        ROS_INFO("Reconfigure request : Kv_y = %.3f ", Kvel_y_);
    }
    else if(Kvel_z_ != config.Kv_z) {
        Kvel_z_ = config.Kv_z;
        ROS_INFO("Reconfigure request : Kv_z = %.3f ", Kvel_z_);
    }

    /* References */
    // center
    else if(c_x_ != config.c_x) {
        c_x_ = config.c_x;
        ROS_INFO("Reconfigure request : c_x = %.3f ", c_x_);
    }
    else if(c_x_ != config.c_y) {
        c_y_ = config.c_y;
        ROS_INFO("Reconfigure request : c_y = %.3f ", c_y_);
    }
    else if(c_z_ != config.c_z) {
        c_z_ = config.c_z;
        ROS_INFO("Reconfigure request : c_z = %.3f ", c_z_);
    }
    // radium
    else if(r_x_ != config.r_x) {
        r_x_ = config.r_x;
        ROS_INFO("Reconfigure request : r_x = %.3f ", r_x_);
    }
    else if(r_y_ != config.r_y) {
        r_y_ = config.r_y;
        ROS_INFO("Reconfigure request : r_y = %.3f ", r_y_);
    }
    else if(r_z_ != config.r_z) {
        r_z_ = config.r_z;
        ROS_INFO("Reconfigure request : r_z = %.3f ", r_z_);
    }
    // frequency
    else if(fr_x_ != config.fr_x) {
        fr_x_ = config.fr_x;
        ROS_INFO("Reconfigure request : fr_x = %.3f ", fr_x_);
    }
    else if(fr_y_ != config.fr_y) {
        fr_y_ = config.fr_y;
        ROS_INFO("Reconfigure request : fr_y = %.3f ", fr_y_);
    }
    else if(fr_z_ != config.fr_z) {
        fr_z_ = config.fr_z;
        ROS_INFO("Reconfigure request : fr_z = %.3f ", fr_z_);
    }
    // phase shift
    else if(ph_x_ != config.ph_x) {
        ph_x_ = config.ph_x;
        ROS_INFO("Reconfigure request : ph_x = %.3f ", ph_x_);
    }
    else if(ph_y_ != config.ph_y) {
        ph_y_ = config.ph_y;
        ROS_INFO("Reconfigure request : ph_y = %.3f ", ph_y_);
    }
    else if(ph_z_ != config.ph_z) {
        ph_z_ = config.ph_z;
        ROS_INFO("Reconfigure request : ph_z = %.3f ", ph_z_);
    }

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;  
}

void mrotorCtrl::statusloopCb(const ros::TimerEvent &event) {
    // ROS_INFO_STREAM(diff_t_);
    // printf("diff_t = %.4f", diff_t_);
    if (sim_enabled_) {
        // Enable OFFBoard mode and arm automatically
        // This will only run if the vehicle is simulated
        mavros_msgs::SetMode offb_set_mode;
        arm_cmd_.request.value = true;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if (mav_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
            if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request_ = ros::Time::now();
        } 

        else {
            if (!mav_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
                if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request_ = ros::Time::now();
            }
        }
    }
    pubSystemStatus();
}


void mrotorCtrl::pubSystemStatus() {
    mavros_msgs::CompanionProcessStatus msg;

    msg.header.stamp = ros::Time::now();
    msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
    msg.state = (int)companion_state_;

    system_status_pub_.publish(msg);
}


void mrotorCtrl::pubTargetPose(double x, double y, double z) { // Target Pose in ENU frame
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    target_pose_pub_.publish(target_pose_);
}


Eigen::Vector4d mrotorCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
}


Eigen::Vector3d mrotorCtrl::applyIOSFBLCtrl(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel) {
    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel; 

    Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;

    // Clip acceleration
    if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;

    // Acceleration reference, not used here
    const Eigen::Vector3d a_ref = Eigen::Vector3d::Zero();

    // Rotor drag compensation, not used here
    const Eigen::Vector3d a_rd = Eigen::Vector3d::Zero();

    // Calculate desired acceleration
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

    return a_des;
}


void mrotorCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
    // Reference attitude
    q_des = acc2quaternion(a_des, mavYaw_);
    controller_ -> Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
    bodyrate_cmd.head(3) = controller_->getDesiredRate();
    double thrust_command = controller_->getDesiredThrust().z();
    bodyrate_cmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command + norm_thrust_offset_));  
    //[bug]// controller_->getDesiredThrust()(3); // Calculate thrust 
}


void mrotorCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    if(rate_ctrl_enabled_){
        msg.type_mask = 128;  // Ignore orientation messages
    }
    else {
        msg.type_mask = 1|2|4;
    }
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);
    target_attitude_pub_.publish(msg);
}

void mrotorCtrl::debugRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    if(rate_ctrl_enabled_){
        msg.type_mask = 128;  // Ignore orientation messages
    }
    else {
        msg.type_mask = 1|2|4;
    }
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);
    target_attitude_debug_pub_.publish(msg);
}

void mrotorCtrl::updateReference(){
    double t = ros::Time::now().toSec();
    double sp_x = c_x_ + r_x_ * std::sin(fr_x_ * t + ph_x_);
    double sp_y = c_y_ + r_y_ * std::sin(fr_y_ * t + ph_y_);
    double sp_z = c_z_ + r_z_ * std::sin(fr_z_ * t + ph_z_);
    double sp_x_dt = r_x_ * fr_x_ * std::cos(fr_x_ * t + ph_x_);
    double sp_y_dt = r_y_ * fr_y_ * std::cos(fr_y_ * t + ph_y_);
    double sp_z_dt = r_z_ * fr_z_ * std::cos(fr_z_ * t + ph_z_);

    // Entering tracking
    if(!last_tracking_state_ && traj_tracking_enabled_ && (mav_num_>1 && (std::abs(mavPos_(0)-sp_x)>0.1 || std::abs(mavPos_(1)-sp_y)>0.1))) {
        targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
        targetVel_ << 0.0, 0.0, 0.0;
        targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero  
        // ROS_INFO("1");
    }    

    // Running tracking
    else if(last_tracking_state_ && traj_tracking_enabled_) {
        targetPos_ << sp_x, sp_y, sp_z;
        targetVel_ << sp_x_dt, sp_y_dt, sp_z_dt;
        last_tracking_state_ = traj_tracking_enabled_;
        // ROS_INFO("2");
    }

    // Exiting from tracking
    else if(last_tracking_state_ && (mav_num_>1 && (std::abs(mavPos_(0)-pos_x_0_)>tracking_exit_min_error_ || std::abs(mavPos_(1)-pos_y_0_)>tracking_exit_min_error_))) {
        targetPos_ << sp_x, sp_y, sp_z;
        targetVel_ << sp_x_dt, sp_y_dt, sp_z_dt;   
        // ROS_INFO("3");     
    }

    // Initial Point
    else {
        targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
        targetVel_ << 0.0, 0.0, 0.0;
        targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero  
        last_tracking_state_ = traj_tracking_enabled_;
        // ROS_INFO("4");
    }
    
}

void mrotorCtrl::exeControl() {
    if(init_complete_){
        Eigen::Vector3d desired_acc;
        desired_acc = applyIOSFBLCtrl(targetPos_, targetVel_);
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
