#ifndef SLS_CONTROLLER_H
#define SLS_CONTROLLER_H

#include "mrotor_controller/mrotor_controller.hpp"
#include "controller_msgs/SlsState.h"
#include "controller_msgs/SlsForce.h"
#include "mrotor_controller/KalmanFilter.hpp"
#include "mrotor_controller/LowPassFilter.hpp"

class mrotorSlsCtrl: public mrotorCtrl {
  private:
    /* Publishers */
    ros::Publisher sls_state_pub_;
    ros::Publisher sls_state_raw_pub_;
    ros::Publisher sls_state_ekf_pub_;
    ros::Publisher sls_force_pub_;

    /* Subscribers */
    ros::Subscriber vicon_load_sub_;
    
    /* Messages */
    controller_msgs::SlsState sls_state_raw_; 
    controller_msgs::SlsState sls_state_; 
    controller_msgs::SlsForce sls_force_; 
    
    /* ROS Time */
    ros::Time vicon_load_last_called_;

    /* Vectors */
    Eigen::Vector3d loadPos_, loadVel_, loadAcc_;
    Eigen::Vector3d loadPos_prev_, loadVel_prev_, loadAcc_prev_;
    Eigen::Vector3d loadPosTarget_, loadVelTarget_;
    double loadPosTarget_x_, loadPosTarget_y_, loadPosTarget_z_;
    Eigen::Vector3d targetRadium_, targetFrequency_, targetPhase_;
    
    // unit vector q
    Eigen::Vector3d pendAngle_, pendRate_, pendAngularAcc_;
    Eigen::Vector3d pendAngle_prev_, pendRate_prev_, pendAngularAcc_prev_;

    /* Variables */
    double cable_length_;
    double load_mass_;    

    const char* gazebo_link_name_[1] = {
      "px4vision_0::px4vision_ancl::base_link", 
    };

    Eigen::MatrixXd P_, Q_, R_;
    std::shared_ptr<mrotorSlsEKF> ekf_;

    /* LPFs */
    // double load_vel_tau_up_, load_vel_tau_down_;
    // FirstOrderFilter* load_vel_filter_;

    std::unique_ptr<SecondOrderFilter<Eigen::Vector3d>> load_vel_filter_;
    std::unique_ptr<SecondOrderFilter<Eigen::Vector3d>> pend_rate_filter_;

    /* Callback Functions */
    void cmdloopCb(const ros::TimerEvent &event);
    void gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void viconDrone1PoseCb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void viconDrone2PoseCb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void viconLoad1PoseCb(const geometry_msgs::TransformStamped::ConstPtr& msg);

    /* Helper Functions */
    Eigen::Vector3d applyQuasiSlsCtrl(void);
    void updateReference(void);
    void checkMissionStage(double mission_time_span);
    void exeControl(void);
    Eigen::Vector3d transformPose(Eigen::Vector3d p_0, Eigen::Vector3d offsetVector);
    Eigen::Vector3d compensateRotorDrag(double t);
    void loadSlsState();
    void loadSlsStateRaw();
    void applyIteration(void);
    void applyFiniteDiffSys(void);
    void applyLowPassFilter(void);
    void readViconDronePose(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void readViconLoadPose(const geometry_msgs::TransformStamped::ConstPtr& msg);
    
  public:
    mrotorSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~mrotorSlsCtrl();
  protected:
};

#endif
