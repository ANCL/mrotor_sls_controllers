#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "QSFGeometricController.h"
#include "rtwtypes.h"

#include <ros/ros.h>
#include "controller_msgs/SlsState.h"
#include "controller_msgs/SlsForce.h"
#include "iostream"
#include <Eigen/Dense>
#include "math.h"

#include "mrotor_controller/common.h"
#include "mrotor_controller/control.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"

#ifndef MATH_PI
#define MATH_PI		3.141592653589793238462643383280
#endif

namespace gazebo
{

class SlsForceControlPlugin : public ModelPlugin
{
private:
    // >>> Pointers
    physics::ModelPtr model;
    physics::LinkPtr baseLink, pendulum, load;
    physics::JointPtr pendulumJoint, loadJoint;
    event::ConnectionPtr updateConnection;

    // >>> Link States
    ignition::math::Vector3d mavPos_;
    ignition::math::Vector3d mavVel_;
    ignition::math::Quaterniond mavAtt_;
    ignition::math::Vector3d mavAttEuler_;
    ignition::math::Vector3d mavRate_;
    ignition::math::Vector3d mavAngularAcc_;
    ignition::math::Vector3d loadPos_;
    ignition::math::Vector3d loadVel_;
    ignition::math::Vector3d pendAngle_;
    ignition::math::Vector3d pendRate_;

    // >>> Controller Parameters
    
    double K_[10] = {24, 50, 35, 10, 24, 50, 35, 10, 2, 3};
    // double K_[10] = {24, 50, 10, 5, 24, 50, 10, 5, 2, 3};
    double param_[4] = {0.25, 1.51, 0.85, 9.80665};
    double max_fb_force_;
    double target_force_ned_[3] = {};
    bool use_ref_att_ = false;
    bool use_ref_rate_ = true;
    bool rate_pid_enabled_ = true;
    
    // >>> Attitude Controller
    bool init_complete_ = false;
    Eigen::Vector4d q_des_;
    double mavYaw_ = 0;
    double attctrl_tau_ = 0.1;
    Eigen::Vector4d cmdBodyRate_;
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_thrust_{Eigen::Vector3d::Zero()};

    // >>> Rate PID Controller
    double diff_t_, last_t_;
    Eigen::Vector3d lim_int_;
    Eigen::Vector3d last_rate_;
    double angular_imax_ = 123456789.0;
    double maxTorque_ = 100;
    Eigen::Vector3d rate_pid_gain_k_;
    Eigen::Vector3d rate_pid_gain_p_;
    Eigen::Vector3d rate_pid_gain_i_;
    Eigen::Vector3d rate_pid_gain_d_;
    Eigen::Vector3d rate_pid_gain_ff_;
    Eigen::Vector3d rate_pid_int_;

    Eigen::Vector3d previousRateState_;
    double timeConstantDownRate_;
    double timeConstantUpRate_;
    double fcRate_ = 100;

    Eigen::Vector3d previousRateDState_;
    double timeConstantDownRateD_;
    double timeConstantUpRateD_;
    double fcRateD_ = 30;

    Eigen::Vector4d previousAttState_;
    double timeConstantDownAtt_;
    double timeConstantUpAtt_;
    double fcAtt_ = 30;



    // >>> Mission Stages
    int mission_stage_ = 0;

    // >>> Reference
    // reference
    double target_center_enu_[3] = {0, 0, 1};
    double target_radium_enu_[3] = {1, 1, 0};
    double target_frequency_enu_[3] = {1, 1, 0};
    double target_phase_enu_[3] = {1.57, 0, 0};
 
    // mission setpoints
    double target_center_enu_1_[3] = {0, 0, 1};
    double target_center_enu_2_[3] = {1.5, 0, 1};
    double target_center_enu_3_[3] = {0, 1.5, 1};

    double ref_trajectory_ned_[12] = {
        target_radium_enu_[1], target_frequency_enu_[1], target_center_enu_[1], target_phase_enu_[1],
        target_radium_enu_[0], target_frequency_enu_[0], target_center_enu_[0], target_phase_enu_[0],
        target_radium_enu_[2], target_frequency_enu_[2], -target_center_enu_[2], target_phase_enu_[2]
    };

    double ref_trajectory_ned_1_[12] = {
        0, 0, target_center_enu_1_[1], 0,
        0, 0, target_center_enu_1_[0], 0,
        0, 0, -target_center_enu_1_[2], 0
    };

    double ref_trajectory_ned_2_[12] = {
        0, 0, target_center_enu_2_[1], 0,
        0, 0, target_center_enu_2_[0], 0,
        0, 0, -target_center_enu_2_[2], 0
    };

    double ref_trajectory_ned_3_[12] = {
        0, 0, target_center_enu_3_[1], 0,
        0, 0, target_center_enu_3_[0], 0,
        0, 0, -target_center_enu_3_[2], 0
    };

    Eigen::Vector3d targetJerk_ = Eigen::Vector3d::Zero();


    // >>> Mission
    ros::Time mission_last_called_;
    ros::Time plugin_loaded_ = ros::Time::now();
    bool mission_initialized_ = false;
    


    // >>> ROS Interface
    ros::Publisher sls_state_pub_;
    ros::Publisher sls_force_pub_;
    ros::Publisher mav_att_pub_;
    ros::Publisher mav_att_sp_pub_;
    ros::Publisher mav_rate_pub_;
    ros::Publisher mav_rate_sp_pub_;
    controller_msgs::SlsState sls_state_; 
    controller_msgs::SlsForce sls_force_; 
    geometry_msgs::QuaternionStamped mav_att_sp_, mav_att_; 
    geometry_msgs::Vector3Stamped mav_rate_sp_, mav_rate_;


public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
        
        ROS_INFO_STREAM("ROS Plugin");
        
        int argc = 0; 
        char **argv = nullptr; 
        ros::init(argc, argv, "gazebo_force_control_node", ros::init_options::NoSigintHandler); 
        ROS_INFO_STREAM("Gazebo Force Control Plugin Initialized");

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        sls_state_pub_ = nh.advertise<controller_msgs::SlsState> ("/drone1/mrotor_sls_controller/sls_state", 1);
        sls_force_pub_ = nh.advertise<controller_msgs::SlsForce> ("/drone1/mrotor_sls_controller/sls_force", 1);
        mav_att_pub_ = nh.advertise<geometry_msgs::QuaternionStamped> ("/drone1/mrotor_sls_controller/mav_att", 1);
        mav_att_sp_pub_ = nh.advertise<geometry_msgs::QuaternionStamped> ("/drone1/mrotor_sls_controller/mav_att_sp", 1);
        mav_rate_pub_ = nh.advertise<geometry_msgs::Vector3Stamped> ("/drone1/mrotor_sls_controller/mav_rate", 1);
        mav_rate_sp_pub_ = nh.advertise<geometry_msgs::Vector3Stamped> ("/drone1/mrotor_sls_controller/mav_rate_sp", 1);

        model = _model;

        // Get links
        baseLink = model->GetLink("base_link");
        pendulum = model->GetLink("pendulum");
        load = model->GetLink("load");

        // Get joints
        pendulumJoint = model->GetJoint("pendulum_joint");
        loadJoint = model->GetJoint("load_joint");

        if (!baseLink || !pendulum || !load || !pendulumJoint || !loadJoint)
        {
            gzerr << "Required links or joints not found in the model!\n";
            return;
        }

        for (int i = 0; i < 10; i++) {
            std::cout << "K_[" << i << "]: " << K_[i] << std::endl;
        }

        std::cout << "Load Mass: " << param_[0] << std::endl;
        std::cout << "Drone Mass: " << param_[1] << std::endl;
        std::cout << "Cable Length: " << param_[2] << std::endl;
        std::cout << "Gravity: " << param_[3] << std::endl;

        max_fb_force_ = 3 * param_[1] * param_[3];
        std::cout << "Max Force: " << max_fb_force_ << std::endl;

        // Rate PID Controller
        lim_int_ << 0.3, 0.3, 0.3;

        rate_pid_gain_k_ << 0.85, 0.85, 1;
        rate_pid_gain_p_ << 0.15, 0.15, 0.30;
        rate_pid_gain_i_ << 0.1, 0.1, 0.1;
        rate_pid_gain_d_ << 0.0004, 0.0004, 0.0;

        rate_pid_gain_k_ << 1.0, 1.0, 1.0;
        rate_pid_gain_p_ << 1.5, 1.5, 3.0;
        rate_pid_gain_i_ << 0.1, 0.1, 0.1;
        rate_pid_gain_d_ << 0.004, 0.004, 0.0;

        rate_pid_gain_p_ = rate_pid_gain_p_.cwiseProduct(rate_pid_gain_k_);
        rate_pid_gain_i_ = rate_pid_gain_i_.cwiseProduct(rate_pid_gain_k_);
        rate_pid_gain_d_ = rate_pid_gain_d_.cwiseProduct(rate_pid_gain_k_);


        ROS_INFO_STREAM("rate_pid_gain_p_" << rate_pid_gain_p_(0) << " " << rate_pid_gain_p_(1) << " " << rate_pid_gain_p_(2));
        ROS_INFO_STREAM("rate_pid_gain_i_" << rate_pid_gain_i_(0) << " " << rate_pid_gain_i_(1) << " " << rate_pid_gain_i_(2));
        ROS_INFO_STREAM("rate_pid_gain_d_" << rate_pid_gain_d_(0) << " " << rate_pid_gain_d_(1) << " " << rate_pid_gain_d_(2));

        timeConstantDownRate_ = 1.d / (2*MATH_PI*fcRate_);
        timeConstantUpRate_ = 1.d / (2*MATH_PI*fcRate_);
        timeConstantDownRateD_ = 1.d / (2*MATH_PI*fcRateD_);
        timeConstantUpRateD_ = 1.d / (2*MATH_PI*fcRateD_);        
        timeConstantDownAtt_ = 1.d / (2*MATH_PI*fcAtt_);
        timeConstantUpAtt_ = 1.d / (2*MATH_PI*fcAtt_);   

        // Connect to the update event
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SlsForceControlPlugin::OnUpdate, this));

        init_complete_ = true;
        last_t_ = ros::Time::now().toSec();
        ROS_INFO_STREAM("Init Complete");
    }

    void OnUpdate() {
        // Time step
        diff_t_ = ros::Time::now().toSec() - last_t_;
        last_t_ = ros::Time::now().toSec();
        // Retrieve base link (quadrotor) states
        mavPos_ = baseLink->WorldPose().Pos();
        mavAtt_ = baseLink->WorldPose().Rot();
        mavAttEuler_ = mavAtt_.Euler();
        // ROS_INFO_STREAM(mavAttEuler_.X() << " " << mavAttEuler_.Y() << " " << mavAttEuler_.Z());
        // mavYaw_ = mavAttEuler_.Z();
        mavVel_ = baseLink->WorldLinearVel();
        mavRate_ = baseLink->WorldAngularVel();
        mavAngularAcc_ = baseLink->RelativeAngularAccel();
        // ignition::math::Quaterniond quadOrientation = baseLink->WorldPose().Rot();

        // Retrieve load states
        loadPos_ = load->WorldPose().Pos();
        loadVel_ = load->WorldLinearVel();

        // Calculate pendulum angles (alpha, beta)
        pendAngle_ = (loadPos_ - mavPos_);
        pendAngle_ = pendAngle_.Normalize();
        pendRate_ = pendAngle_.Cross(loadVel_ - mavVel_);

        sls_state_.header.stamp = ros::Time::now();
        sls_state_.sls_state[0] = loadPos_.Y();
        sls_state_.sls_state[1] = loadPos_.X();
        sls_state_.sls_state[2] = -loadPos_.Z();
        sls_state_.sls_state[3] = pendAngle_.Y();
        sls_state_.sls_state[4] = pendAngle_.X();
        sls_state_.sls_state[5] = -pendAngle_.Z();
        sls_state_.sls_state[6] = loadVel_.Y();
        sls_state_.sls_state[7] = loadVel_.X();
        sls_state_.sls_state[8] = -loadVel_.Z(); 
        sls_state_.sls_state[9] = pendRate_.Y();
        sls_state_.sls_state[10] = pendRate_.X();
        sls_state_.sls_state[11] = -pendRate_.Z();
        sls_state_pub_.publish(sls_state_);

        double sls_state_array[12];
        for(int i=0; i<12;i++){
            sls_state_array[i] = sls_state_.sls_state[i];
        }    

        if(ros::Time::now().toSec() - plugin_loaded_.toSec() < 10) {
            QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
        }

        else {
            switch(mission_stage_) {
            case 0:
                if(!mission_initialized_){
                    ROS_INFO("[exeMission] Mission started at case 0");
                    mission_initialized_ = true;
                    mission_last_called_ = ros::Time::now();
                }
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                checkMissionStage(10);
                break;

            case 1:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                checkMissionStage(10);
                break;

            case 2:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_2_, 0, target_force_ned_);
                checkMissionStage(10);
                break;
            
            case 3:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_3_, 0, target_force_ned_);
                checkMissionStage(10);
                break;

            case 4:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                checkMissionStage(10);
                break;

            case 5:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_, ros::Time::now().toSec() - mission_last_called_.toSec(), target_force_ned_);
                checkMissionStage(20);
                break;                

            default:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                if(ros::Time::now().toSec() - mission_last_called_.toSec() >= 10){
                    ROS_INFO("[exeMission] Mission Accomplished");
                    mission_last_called_ = ros::Time::now();
                }
                break;
            }
        }

        sls_force_.header.stamp = ros::Time::now();
        sls_force_.sls_force[0] = target_force_ned_[0];
        sls_force_.sls_force[1] = target_force_ned_[1];
        sls_force_.sls_force[2] = target_force_ned_[2];
        sls_force_pub_.publish(sls_force_);

        ignition::math::Vector3d controlForce(target_force_ned_[1], target_force_ned_[0], -target_force_ned_[2]);
        if (controlForce.Length() > max_fb_force_)
            controlForce = (max_fb_force_ / controlForce.Length()) * controlForce;
        // this->baseLink->AddForce(controlForce);
        
        // >>> Attitude Control
        Eigen::Vector3d targetForce(controlForce.X(), controlForce.Y(), controlForce.Z());
        // ROS_INFO_STREAM(targetForce(0) << " " << targetForce(1) << " " << targetForce(2));
        Eigen::Vector3d targetAcc = targetForce / param_[1];
        q_des_ = acc2quaternion(targetAcc, mavYaw_);
        Eigen::Vector4d mavAtt(mavAtt_.W(), mavAtt_.X(), mavAtt_.Y(), mavAtt_.Z());
        // mavAtt = updateAttLPF(mavAtt, diff_t_);
        updateAttitudeCtrl(mavAtt, q_des_, targetAcc, targetJerk_);
        cmdBodyRate_.head(3) = getDesiredRate();
        double thrust_command = getDesiredThrust().z();
        cmdBodyRate_(3) = thrust_command * param_[1];

        // >>> Rate Control
        Eigen::Vector3d mavRate(mavRate_.X(), mavRate_.Y(), mavRate_.Z());
        // mavRate = updateRateLPF(mavRate, diff_t_);
        Eigen::Vector3d desired_torque;
        desired_torque = updateRateCtrl(mavRate, cmdBodyRate_.head(3));
        

        if(use_ref_rate_){
            this->baseLink->AddRelativeForce(ignition::math::Vector3d(0, 0, cmdBodyRate_(3)));  
            if(!rate_pid_enabled_) {
                ROS_INFO_STREAM("Instantaneous Rate Control");
                ROS_INFO_STREAM(cmdBodyRate_(0) << " " << cmdBodyRate_(1) << " " << cmdBodyRate_(2) << " " << cmdBodyRate_(3));  
                this->baseLink->SetAngularVel({cmdBodyRate_(0), cmdBodyRate_(1), cmdBodyRate_(2)}); 
            }        

            else {
                // ROS_INFO_STREAM("PID Rate Control");
                // ROS_INFO_STREAM("diff_t_:" << diff_t_);
                // ROS_INFO_STREAM("int:" << rate_pid_int_(0) << " " << rate_pid_int_(1) << " " << rate_pid_int_(2));
                // ROS_INFO_STREAM("torque:" << desired_torque(0) << " " << desired_torque(1) << " " << desired_torque(2));
                this->baseLink->AddRelativeTorque(ignition::math::Vector3d(desired_torque(0), desired_torque(1), desired_torque(2)));
            }
        }

        else if(use_ref_att_) {

        }

        else { // outer-loop control
            ROS_INFO_STREAM("Force Control");
            this->baseLink->AddForce(controlForce);
        }

        mav_att_.header.stamp = ros::Time::now();
        mav_att_.quaternion.x = mavAtt(1);
        mav_att_.quaternion.y = mavAtt(2);
        mav_att_.quaternion.z = mavAtt(3);
        mav_att_.quaternion.w = mavAtt(0);; 
        mav_att_pub_.publish(mav_att_);

        mav_att_sp_.header.stamp = ros::Time::now();
        mav_att_sp_.quaternion.x = q_des_(1);
        mav_att_sp_.quaternion.y = q_des_(2);
        mav_att_sp_.quaternion.z = q_des_(3);
        mav_att_sp_.quaternion.w = q_des_(0);
        mav_att_sp_pub_.publish(mav_att_sp_);

        mav_rate_.header.stamp = ros::Time::now();
        mav_rate_.vector.x = mavRate_.X();
        mav_rate_.vector.y = mavRate_.Y();
        mav_rate_.vector.z = mavRate_.Z();
        mav_rate_pub_.publish(mav_rate_);

        mav_rate_sp_.header.stamp = ros::Time::now();
        mav_rate_sp_.vector.x = cmdBodyRate_(0);
        mav_rate_sp_.vector.y = cmdBodyRate_(1);
        mav_rate_sp_.vector.z = cmdBodyRate_(2);
        mav_rate_sp_pub_.publish(mav_rate_sp_);
    }

    void checkMissionStage(double mission_time_span) {
        if(ros::Time::now().toSec() - mission_last_called_.toSec() >= mission_time_span) {
            mission_last_called_ = ros::Time::now();
            mission_stage_ += 1;
            ROS_INFO_STREAM("[exeMission] Stage " << mission_stage_-1 << " ended, switching to stage " << mission_stage_);
        }
    }

    void updateAttitudeCtrl(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                      const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
        // Geometric attitude controller
        // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
        // attitude control: Technical report. ETH Zurich, 2013.

        const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
        const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
        const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
        desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
        desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
        desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
        const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
        const Eigen::Vector3d zb = rotmat.col(2);
        desired_thrust_(0) = 0.0;
        desired_thrust_(1) = 0.0;
        desired_thrust_(2) = ref_acc.dot(zb);
    }

    Eigen::Vector3d updateRateCtrl(Eigen::Vector3d &curr_rate, const Eigen::Vector3d &ref_rate) {
        // Compute rate error
        Eigen::Vector3d rate_error = ref_rate - curr_rate;

        // Compute angular acceleration
        Eigen::Vector3d mavAngularAcc(mavAngularAcc_.X(), mavAngularAcc_.Y(), mavAngularAcc_.Z());
        // if(diff_t_ > 0.0){
        //     mavAngularAcc = (curr_rate - last_rate_) / diff_t_;
        //     last_rate_ = curr_rate;
        //     // mavAngularAcc = updateRateDLPF(mavAngularAcc, diff_t_);
        // }

        // Update integral
        for(int i = 0; i < 3; i++) {
            double i_factor = rate_error(i) / (400.d * MATH_PI / 180);
            i_factor = std::max(0.0d, 1.d - i_factor * i_factor);
            double rate_i = rate_pid_int_(i) + i_factor * rate_pid_gain_i_(i) * rate_error(i) * diff_t_;
            if (std::isfinite(rate_i)) {
			    rate_pid_int_(i) = (rate_i < -lim_int_(i)) ? -lim_int_(i) : ((rate_i > lim_int_(i)) ? lim_int_(i) : rate_i);
		    }
        }

        // Compute reference torque
        Eigen::Vector3d desired_torque = rate_pid_gain_p_.cwiseProduct(rate_error) + rate_pid_int_ - rate_pid_gain_d_.cwiseProduct(mavAngularAcc) + rate_pid_gain_ff_.cwiseProduct(ref_rate);
        return desired_torque;
    }

    Eigen::Vector3d getDesiredThrust() { return desired_thrust_; };
    Eigen::Vector3d getDesiredRate() { return desired_rate_; };

    ~SlsForceControlPlugin() {

    }


    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
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


    Eigen::Vector4d updateAttLPF(Eigen::Vector4d inputState, double samplingTime) {
        /*
        This method will apply a first order filter on the inputState.
        */
        Eigen::Vector4d outputState;

        for(int i = 0; i < 4; i ++) {
            if(inputState(i) > previousAttState_(i)){
                // Calcuate the outputState if accelerating.
                double alphaUp = exp(- samplingTime / timeConstantUpAtt_);
                // x(k+1) = Ad*x(k) + Bd*u(k)
                outputState(i) = alphaUp * previousAttState_(i) + (1 - alphaUp) * inputState(i);
            }
            
            else{
                // Calculate the outputState if decelerating.
                double alphaDown = exp(- samplingTime / timeConstantDownAtt_);
                outputState(i) = alphaDown * previousAttState_(i) + (1 - alphaDown) * inputState(i);
            }
        }

        previousAttState_ = outputState;
        return outputState;
    }

    Eigen::Vector3d updateRateLPF(Eigen::Vector3d inputState, double samplingTime) {
        /*
        This method will apply a first order filter on the inputState.
        */
        Eigen::Vector3d outputState;

        for(int i = 0; i < 3; i ++) {
            if(inputState(i) > previousRateState_(i)){
                // Calcuate the outputState if accelerating.
                double alphaUp = exp(- samplingTime / timeConstantUpRate_);
                // x(k+1) = Ad*x(k) + Bd*u(k)
                outputState(i) = alphaUp * previousRateState_(i) + (1 - alphaUp) * inputState(i);
            }
            
            else{
                // Calculate the outputState if decelerating.
                double alphaDown = exp(- samplingTime / timeConstantDownRate_);
                outputState(i) = alphaDown * previousRateState_(i) + (1 - alphaDown) * inputState(i);
            }
        }

        previousRateState_ = outputState;
        return outputState;
    }

    Eigen::Vector3d updateRateDLPF(Eigen::Vector3d inputState, double samplingTime) {
        /*
        This method will apply a first order filter on the inputState.
        */
        Eigen::Vector3d outputState;

        for(int i = 0; i < 3; i ++) {
            if(inputState(i) > previousRateState_(i)){
                // Calcuate the outputState if accelerating.
                double alphaUp = exp(- samplingTime / timeConstantUpRateD_);
                // x(k+1) = Ad*x(k) + Bd*u(k)
                outputState(i) = alphaUp * previousRateDState_(i) + (1 - alphaUp) * inputState(i);
            }
            
            else{
                // Calculate the outputState if decelerating.
                double alphaDown = exp(- samplingTime / timeConstantDownRateD_);
                outputState(i) = alphaDown * previousRateDState_(i) + (1 - alphaDown) * inputState(i);
            }
        }

        previousRateState_ = outputState;
        return outputState;

    }


};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SlsForceControlPlugin)


} 