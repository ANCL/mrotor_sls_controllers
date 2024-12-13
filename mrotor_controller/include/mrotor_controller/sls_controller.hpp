#ifndef SLS_CONTROLLER_H
#define SLS_CONTROLLER_H

#include "mrotor_controller/mrotor_controller.hpp"
#include "controller_msgs/SlsState.h"

class mrotorSlsCtrl: public mrotorCtrl {
  private:
    /* Publishers */
    ros::Publisher sls_state_pub_;

    /* Subscribers */
    ros::Subscriber vicon_load_sub_;
    
    /* Messages */
    controller_msgs::SlsState sls_state_; 

    /* Vectors */
    Eigen::Vector3d loadPos_, loadVel_;
    Eigen::Vector3d loadPosTarget_, loadVelTarget_;
    double loadPosTarget_x_, loadPosTarget_y_, loadPosTarget_z_;
    Eigen::Vector3d loadPos_prev_, loadVel_prev_;
    // alpha beta
    Eigen::Vector2d pendAngle_, pendRate_;
    Eigen::Vector2d pendAngle_last_, pendRate_last_;
    // unit vector q
    Eigen::Vector3d pendAngle_q_, pendRate_q_;
    Eigen::Vector3d pendAngle_q_prev_, pendRate_q_prev_;

    /* Variables */
    double cable_length_;
    double load_mass_;    

    /* Callback Functions */
    void gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void viconDrone1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void viconDrone2Cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void viconLoad1Cb(const geometry_msgs::TransformStamped::ConstPtr& msg);

    /* Helper Functions */
    Eigen::Vector3d applyQuasiSlsCtrl();
    void exeControl(void);
    
  public:
    mrotorSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~mrotorSlsCtrl();
  protected:
};

#endif
