#ifndef SLS_CONTROLLER_H
#define SLS_CONTROLLER_H

#include "mrotor_controller/mrotor_controller.hpp"
#include "controller_msgs/SlsState.h"
#include "controller_msgs/SlsForce.h"

class mrotorSlsCtrl: public mrotorCtrl {
  private:
    /* Publishers */
    ros::Publisher sls_state_pub_;
    ros::Publisher sls_force_pub_;

    /* Subscribers */
    ros::Subscriber vicon_load_sub_;
    
    /* Messages */
    controller_msgs::SlsState sls_state_; 
    controller_msgs::SlsForce sls_force_; 

    /* Vectors */
    Eigen::Vector3d loadPos_, loadVel_;
    Eigen::Vector3d loadPos_prev_, loadVel_prev_;
    Eigen::Vector3d loadPosTarget_, loadVelTarget_;
    double loadPosTarget_x_, loadPosTarget_y_, loadPosTarget_z_;
    Eigen::Vector3d targetRadium_, targetFrequency_, targetPhase_;
    
    // unit vector q
    Eigen::Vector3d pendAngle_, pendRate_;
    Eigen::Vector3d pendAngle_prev_, pendRate_prev_;

    /* Variables */
    double cable_length_;
    double load_mass_;    

    const char* gazebo_link_name_[1] = {
      "px4vision_0::px4vision_ancl::base_link", 
    };

    /* Callback Functions */
    void cmdloopCb(const ros::TimerEvent &event);
    void gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void viconDrone1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void viconDrone2Cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void viconLoad1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg);

    /* Helper Functions */
    Eigen::Vector3d applyQuasiSlsCtrl(void);
    void updateReference(void);
    void checkMissionStage(double mission_time_span);
    void exeControl(void);
    
  public:
    mrotorSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~mrotorSlsCtrl();
  protected:
};

#endif
