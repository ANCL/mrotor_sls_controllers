#ifndef MROTOR_CONTROLLER_H
#define MROTOR_CONTROLLER_H

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <mrotor_controller/MrotorControllerConfig.h>

#include "mrotor_controller/common.h"
#include "mrotor_controller/control.h"

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATION,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class mrotorCtrl {
  public:
    /* Node Handles */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    /* Subscribers */
    ros::Subscriber mav_state_sub_;
    ros::Subscriber gazebo_link_state_sub_;
    ros::Subscriber vicon_sub_;

    /* Publishers */
    ros::Publisher target_pose_pub_;
    ros::Publisher target_attitude_pub_;
    ros::Publisher target_attitude_debug_pub_;
    ros::Publisher system_status_pub_;

    /* Service Clients */
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    /* Timers */
    ros::Timer cmdloop_timer_, statusloop_timer_;

    /* ROS Times */
    ros::Time last_request_, last_stage_;
    ros::Time gazebo_last_called_, vicon_last_called_;

    /* Messages */
    mavros_msgs::State mav_state_;
    mavros_msgs::CommandBool arm_cmd_;
    geometry_msgs::PoseStamped target_pose_;

    /* Vectors */
    Eigen::Vector3d mavPos_, mavVel_, mavRate_;
    Eigen::Vector3d mavPos_prev_, mavVel_prev_, mavRate_prev_;
    Eigen::Vector4d mavAtt_, q_des;
    double mavYaw_;
    Eigen::Vector3d targetJerk_;
    Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.8)};
    Eigen::Vector3d Kpos_, Kvel_, D_;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_, Kacc_x_, Kacc_y_, Kacc_z_, Kjer_x_, Kjer_y_, Kjer_z_;
    double pos_x_0_, pos_y_0_, pos_z_0_;
    // Control Targets
    Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJer_, targetSna_;
    // Control Commands
    Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}

    /* MAV_STATE */
    MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;      

    /* Constants */
    double gravity_acc_ = 9.80665;
    double pi = 3.1415926535;
    const char* gazebo_link_name_[1] = {
      "px4vision_0::base_link", 
    };

    /* Variables */
    /* MAV Numbers & ID */ 
    int mav_num_;
    int mav_id_;
    // booleans 
    bool ctrl_enabled_;
    bool rate_ctrl_enabled_;
    bool sim_enabled_;
    bool traj_tracking_enabled_;
    bool init_complete_;
    bool finite_diff_enabled_;
    bool sitl_multi_vehicle_enabled_;
    bool gazebo_link_name_matched_ = false;
    bool last_tracking_state_;
    bool qsf_geo_ctrl_enabled_ = false;
    // gazebo link indices
    int drone_link_index_;
    // number of gazebo links
    int gazebo_link_num_; 
    // drone physical parameters        
    double mav_mass_;
    double max_fb_acc_;
    // // attitude control
    // double attctrl_tau_;
    // throttle normalization
    double max_thrust_force_;
    double norm_thrust_const_;
    double norm_thrust_offset_;
    // periods
    double diff_t_;
    // mission stages
    int stage = 0;
    //reference
    double c_x_, c_y_, c_z_;    // center of trajectory tracking & set-point of static tracking
    double r_x_, r_y_, r_z_;    // radium
    double fr_x_, fr_y_, fr_z_; // frequency
    double ph_x_, ph_y_, ph_z_; // phase shift
    // tolerance
    double tracking_exit_min_error_;

    /* Shared Pointer */
    std::shared_ptr<Control> controller_;

    /* Functions */
    // Callback Functions
    void mavstateCb(const mavros_msgs::State::ConstPtr& msg);
    void gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void statusloopCb(const ros::TimerEvent &event);
    void viconCb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void dynamicReconfigureCb(mrotor_controller::MrotorControllerConfig &config, uint32_t level);
    // Helper Functions
    void pubSystemStatus();
    void pubTargetPose(double x, double y, double z); // Target Pose in ENU frame
    void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
    void debugRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    Eigen::Vector3d applyIOSFBLCtrl(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel);
    void updateReference();
    void exeControl();
    // APIs for Child Classes

  // public:
    mrotorCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~mrotorCtrl();

  private:
  protected:

};

#endif