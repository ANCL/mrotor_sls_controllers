#include "mrotor_controller/sls_controller.hpp"

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

    sls_state_.header.stamp = ros::Time::now();
    sls_state_.sls_state[0] = mavPos_(0);
    sls_state_.sls_state[1] = -mavPos_(1);
    sls_state_.sls_state[2] = -mavPos_(2);
    sls_state_.sls_state[3] = pendAngle_q_(0);
    sls_state_.sls_state[4] = -pendAngle_q_(1);
    sls_state_.sls_state[5] = -pendAngle_q_(2);
    sls_state_.sls_state[6] = mavVel_(0);
    sls_state_.sls_state[7] = -mavVel_(1);
    sls_state_.sls_state[8] = -mavVel_(2); 
    sls_state_.sls_state[9] = pendRate_q_(0);
    sls_state_.sls_state[10] = -pendRate_q_(1);
    sls_state_.sls_state[11] = -pendRate_q_(2);

    /* Publish Control Commands*/
    exeControl();

    // exeMission();
}

Eigen::Vector3d mrotorSlsCtrl::applyQuasiSlsCtrl(){

}

void mrotorSlsCtrl::exeControl(void){

}

void mrotorSlsCtrl::viconDrone1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg){

}

void mrotorSlsCtrl::viconDrone2Cb(const geometry_msgs::TransformStamped::ConstPtr& msg){

}

void mrotorSlsCtrl::viconLoad1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg){

}