#include "Eigen/Dense"
#include <ros/ros.h>
#include "controller_msgs/SlsState.h"
#include "controller_msgs/SlsForce.h"

class mrotorSlsEKF{
    /* Extended Kalman Filter */
    public:
        mrotorSlsEKF(double mav_mass, double load_mass, double cable_length, Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R, bool verbose): 
            m__q(mav_mass), m__p(load_mass), L(cable_length), P_(P), Q_(Q), R_(R), verbose_(verbose) {
            gravity_ << 0, 0, 9.80665;
            optimalState_ = Eigen::VectorXd::Zero(12);
            if(verbose_) {
                ROS_INFO_STREAM("EKF Init");
                std::cout << "[EKF] P: " << std::endl << P_ << std::endl;
                std::cout << "[EKF] Q: " << std::endl << Q_ << std::endl;
                std::cout << "[EKF] R: " << std::endl << R_ << std::endl;
            }
            ROS_INFO_STREAM("optim0: " << optimalState_(0));
        }

        ~mrotorSlsEKF() {}

        controller_msgs::SlsState updateFilter(controller_msgs::SlsState inputStateMsg, controller_msgs::SlsForce inputForceMsg, double samplingTime) {
            // ROS_INFO_STREAM("1: " << optimalState_);
            if(verbose_) {
                ROS_INFO_STREAM("Updating Filter ... ");
            }
            updatePrediction(inputForceMsg, samplingTime);
            // ROS_INFO_STREAM("2: " << optimalState_);
            updateMeasurement(inputStateMsg);
            // ROS_INFO_STREAM("3: " << optimalState_);

            controller_msgs::SlsState outputState;
            outputState.header.stamp = ros::Time::now();
            outputState.sls_state[0] = optimalState_(0);
            outputState.sls_state[1] = optimalState_(1);
            outputState.sls_state[2] = optimalState_(2);
            outputState.sls_state[3] = optimalState_(3);
            outputState.sls_state[4] = optimalState_(4);
            outputState.sls_state[5] = optimalState_(5);
            outputState.sls_state[6] = optimalState_(6);
            outputState.sls_state[7] = optimalState_(7);
            outputState.sls_state[8] = optimalState_(8);
            outputState.sls_state[9] = optimalState_(9);
            outputState.sls_state[10] = optimalState_(10);
            outputState.sls_state[11] = optimalState_(11);
            return outputState;

        }

        void updatePrediction(controller_msgs::SlsForce inputForceMsg, double samplingTime) {
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Updating Prediction ... ");
            }
            const Eigen::Vector3d loadPose = optimalState_.head(3);
            const Eigen::Vector3d pendAngle = optimalState_.segment(3, 3);
            const Eigen::Vector3d loadVel = optimalState_.segment(6, 3);
            const Eigen::Vector3d pendRate = optimalState_.tail(3);
            double F1 = inputForceMsg.sls_force[0];
            double F2 = inputForceMsg.sls_force[1];
            double F3 = inputForceMsg.sls_force[2];
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Computing inputU ... ");
            }            
            Eigen::Vector3d inputU;
            inputU << 
                (-L*m__q*pendAngle(0)*pendAngle(2)*pendRate(0)*pendRate(0) - L*m__q*pendAngle(0)*pendAngle(2)*pendRate(1)*pendRate(1) - L*m__q*pendAngle(0)*pendAngle(2)*pendRate(2)*pendRate(2) + F1*m__p*pendAngle(0)*pendAngle(2) + F1*m__q*pendAngle(0)*pendAngle(2) + F2*m__p*pendAngle(1)*pendAngle(2) + F2*m__q*pendAngle(1)*pendAngle(2) - F3*m__p*pendAngle(0)*pendAngle(0) - F3*m__p*pendAngle(1)*pendAngle(1) - F3*m__q*pendAngle(0)*pendAngle(0) - F3*m__q*pendAngle(1)*pendAngle(1) + F3*m__p + F3*m__q)*1/pendAngle(2)*1/(m__p*m__p + (2*m__p)*m__q + m__q*m__q),
                F2*pendAngle(2) - F3*pendAngle(1),
                (-L*m__q*pendAngle(2)*pendRate(0)*pendRate(0) - L*m__q*pendAngle(2)*pendRate(1)*pendRate(1) - L*m__q*pendAngle(2)*pendRate(2)*pendRate(2) + F1*m__p*pendAngle(2) + F1*m__q*pendAngle(2) - F3*m__p*pendAngle(0) - F3*m__q*pendAngle(0))*1/(m__q + m__p);
            ROS_INFO_STREAM("inputU: " << inputU);
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Computing A ... ");
            }  
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12), B = Eigen::MatrixXd::Zero(12, 3);

            A(4, 5) = -pendRate(0);
            A(5, 4) = -pendRate(0);
            A(3, 5) = pendRate(1);
            A(5, 3) = -pendRate(1);
            A(3, 4) = -pendRate(2); 
            A(4, 3) = pendRate(2);

            A(0, 6) = 1;
            A(1, 7) = 1;
            A(2, 8) = 1;

            A(3, 10) = pendAngle(2);
            A(3, 11) = -pendAngle(1);
            A(4, 9) = -pendAngle(2);
            A(4, 11) = pendAngle(0);
            A(5, 9) = pendAngle(1);
            A(5, 10) = -pendAngle(0);

            A(6, 3) = inputU(0);
            A(7, 4) = inputU(0);
            A(8, 5) = inputU(0);

            A(11, 3) = -inputU(1) / (m__q * L * pendAngle(2));
            A(11, 4) = -inputU(2) / (m__q * L * pendAngle(2));
            A(11, 5) = inputU(1) * pendAngle(0) / (m__q * L * pendAngle(2) * pendAngle(2)) + inputU(2) * pendAngle(1) / (m__q * L * pendAngle(2) * pendAngle(2));


            B(6, 0) = pendAngle(0);
            B(7, 0) = pendAngle(1);
            B(8, 0) = pendAngle(2);

            B(9, 1) = 1 / (m__q * L);
            B(10, 2) = 1 / (m__q * L);
            B(11, 1) = - pendAngle(0) / (m__q * L * pendAngle(2));
            B(11, 2) = - pendAngle(1) / (m__q * L * pendAngle(2));

            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Updating estimated state and covariance ... ");
            } 
            optimalState_ += (A * optimalState_ + B * inputU) * samplingTime;
            P_ = A*P_*A.transpose() + Q_;

            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Prediction Complete");
                std::cout << "[EKF] A: " << std::endl << A << std::endl;
                std::cout << "[EKF] B: " << std::endl << B << std::endl;
            }
            
            ROS_INFO_STREAM("P_1: " << P_(1, 1));
            ROS_INFO_STREAM("optim1: " << optimalState_(0));
        }


        void updateMeasurement(controller_msgs::SlsState inputStateMsg) {
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Updating measurement ... ");
            } 
            const Eigen::Vector3d loadPose(inputStateMsg.sls_state[0], inputStateMsg.sls_state[1], inputStateMsg.sls_state[2]);
            const Eigen::Vector3d pendAngle(inputStateMsg.sls_state[3], inputStateMsg.sls_state[4], inputStateMsg.sls_state[5]);
            const Eigen::Vector3d loadVel(inputStateMsg.sls_state[6], inputStateMsg.sls_state[7], inputStateMsg.sls_state[8]);
            const Eigen::Vector3d pendRate(inputStateMsg.sls_state[9], inputStateMsg.sls_state[10], inputStateMsg.sls_state[11]);
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Setting input state ... ");
            } 
            Eigen::VectorXd inputState(loadPose.size() + pendAngle.size() + loadVel.size() + pendRate.size());
            inputState << loadPose, pendAngle, loadVel, pendRate;

            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Setting H ... ");
            } 

            const Eigen::MatrixXd H = Eigen::MatrixXd::Identity(12, 12);
            
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Setting errorState ... ");
            } 

            const Eigen::VectorXd errorState = optimalState_ - H * inputState;

            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Setting S ... ");
            } 

            const Eigen::MatrixXd S = H * P_ * H.transpose() + R_;


            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Setting K ... ");
            } 


            // const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
            const Eigen::MatrixXd K = P_ * H.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();
            ROS_INFO_STREAM("P_2: " << P_(1, 1));
            ROS_INFO_STREAM("S: " << S(1, 1));
            ROS_INFO_STREAM("K: " << K(1, 1));
            ROS_INFO_STREAM("optim2: " << optimalState_(0));

            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Updating Previous Estimation Based on Measurement ... ");
            } 
            optimalState_ += K * errorState;
            // P_ = (Eigen::MatrixXd::Identity(12, 12) - K*H) * P_;
        }

        void resetInitState(controller_msgs::SlsState initStateMsg) {
            if(verbose_) {
                ROS_INFO_STREAM("[EKF] Resetting Initial State ... ");
                ROS_INFO_STREAM(initStateMsg);
            }
            const Eigen::Vector3d loadPose(initStateMsg.sls_state[0], initStateMsg.sls_state[1], initStateMsg.sls_state[2]);
            const Eigen::Vector3d pendAngle(initStateMsg.sls_state[3], initStateMsg.sls_state[4], initStateMsg.sls_state[5]);
            const Eigen::Vector3d loadVel(initStateMsg.sls_state[6], initStateMsg.sls_state[7], initStateMsg.sls_state[8]);
            const Eigen::Vector3d pendRate(initStateMsg.sls_state[9], initStateMsg.sls_state[10], initStateMsg.sls_state[11]);
            if(verbose_) {
                ROS_INFO_STREAM("Setting optimState ... ");
            }
            Eigen::VectorXd optimalState(loadPose.size() + pendAngle.size() + loadVel.size() + pendRate.size());
            optimalState << loadPose, pendAngle, loadVel, pendRate;
            optimalState_ = optimalState;
            ROS_INFO_STREAM("loadpose_msg_reset: "<< initStateMsg.sls_state[0]);
            ROS_INFO_STREAM("loadpose_reset: " << loadPose(0));
            ROS_INFO_STREAM("optim_reset: " << optimalState_(0));
            if(verbose_) {
                ROS_INFO_STREAM("Resetting Complete");
            }
        }



    private:
        double m__q;
        double m__p;
        double L;
        Eigen::Vector3d gravity_;
        Eigen::VectorXd optimalState_;
        Eigen::MatrixXd P_, Q_, R_;
        bool verbose_;

    protected:
};



// loadPose_ += loadVel * samplingTime;
// pendAngle_ += pendRate.cross(pendAngle) * samplingTime;
// loadVel_ += -m__q * L / (m__q + m__p) * pendRate.squaredNorm() * pendAngle + gravity_ + inputForce.dot(pendAngle)*(pendAngle) / (m__q + m__p);  
// pendRate_ += -1/(m__q * L) * pendAngle.cross(inputForce);