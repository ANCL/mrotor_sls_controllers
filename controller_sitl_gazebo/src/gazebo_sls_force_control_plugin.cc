#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include "QSFGeometricController.h"
#include "rtwtypes.h"

#include <ros/ros.h>
#include "controller_msgs/SlsState.h"
#include "controller_msgs/SlsForce.h"

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
    ignition::math::Vector3d loadPos_;
    ignition::math::Vector3d loadVel_;
    ignition::math::Vector3d pendAngle_;
    ignition::math::Vector3d pendRate_;

    // >>> Controller Parameters
    double K_[10] = {24, 50, 35, 10, 24, 50, 35, 10, 2, 3};
    double param_[4] = {0.25, 1.51, 0.85, 9.80665};
    double target_force_ned_[3] = {};

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


    // >>> Mission
    ros::Time mission_last_called_;
    ros::Time plugin_loaded_ = ros::Time::now();
    bool mission_initialized_ = false;
    


    // >>> ROS Interface
    ros::Publisher sls_state_pub_;
    ros::Publisher sls_force_pub_;
    controller_msgs::SlsState sls_state_; 
    controller_msgs::SlsForce sls_force_; 

    // // >>> Logging
    // std::ofstream logTxtFile;
    // std::ofstream logCsvFile;
    // bool isFileInitialized = false;
    
    // // >>> Misc
    // double holdTime = 10.0; // Delay in seconds before activating the Mission
    // // Simulation time
    // double currentSimTime = 0;
    // double timeSinceLastRequest = 0;
    // double lastRequestSimTime = 0;
    // double logCount = 0;
    // double timestep;


public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
        
        ROS_INFO_STREAM("ROS Plugin NO");
        
        int argc = 0; 
        char **argv = nullptr; 
        ros::init(argc, argv, "gazebo_force_control_node", ros::init_options::NoSigintHandler); 
        ROS_INFO_STREAM("Gazebo Force Control Plugin Initialized");

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        sls_state_pub_ = nh.advertise<controller_msgs::SlsState> ("/drone1/mrotor_sls_controller/sls_state", 1);
        sls_force_pub_ = nh.advertise<controller_msgs::SlsForce> ("/drone1/mrotor_sls_controller/sls_force", 1);

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


        // // Delete the existing log file if it exists
        // if (std::filesystem::exists("log.txt"))
        // {
        //     std::filesystem::remove("log.txt");
        // }
        // if (std::filesystem::exists("log.csv"))
        // {
        //     std::filesystem::remove("log.csv");
        // }

        // // Create and open CSV file
        // logTxtFile.open("log.txt", std::ios::out);
        // if (!logTxtFile.is_open())
        // {
        //     gzerr << "Unable to open or create txt file for logging!\n";
        //     return;
        // }
        // logCsvFile.open("log.csv", std::ios::out);
        // if (!logCsvFile.is_open())
        // {
        //     gzerr << "Unable to open or create CSV file for logging!\n";
        //     return;
        // }

        // // Write header to txt if not initialized
        // if (!isFileInitialized)
        // {
        //     logTxtFile << "LOG FILE -- FULL STATES\n";
        //     logCsvFile << "Time,Des_Pos_X,Des_Pos_Y,Des_Pos_Z,Quad_Pos_X,Quad_Pos_Y,Quad_Pos_Z,"
        //            << "Roll,Pitch,Yaw,Vx,Vy,Vz,Lx,Ly,Lz,Alpha,Beta,GammaAlpha,GammaBeta,Fx,Fy,Fz\n";
        //     isFileInitialized = true;
        // }

        for (int i = 0; i < 10; i++) {
            std::cout << "K_[" << i << "]: " << K_[i] << std::endl;
        }

        std::cout << "Load Mass: " << param_[0] << std::endl;
        std::cout << "Drone Mass: " << param_[1] << std::endl;
        std::cout << "Cable Length: " << param_[2] << std::endl;
        std::cout << "Gravity: " << param_[3] << std::endl;

        // // Connect to the update event
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SlsForceControlPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        // Retrieve base link (quadrotor) states
        mavPos_ = baseLink->WorldPose().Pos();
        mavVel_ = baseLink->WorldLinearVel();
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
        this->baseLink->AddForce(controlForce);

        // // Retrieve quadrotor orientation in roll, pitch, yaw
        // roll = quadOrientation.Roll();
        // pitch = quadOrientation.Pitch();
        // yaw = quadOrientation.Yaw();

        // currLoc[0] = quadPosition.X();
        // currLoc[1] = quadPosition.Y();
        // currLoc[2] = quadPosition.Z();

        // timestep = model->GetWorld()->SimTime().Double() - currentSimTime;
        // currentSimTime = model->GetWorld()->SimTime().Double();
        // timeSinceLastRequest = currentSimTime - lastRequestSimTime;

        // switch (stage)
        // {
        // case 0:
        //     Setpoint[0] = 0; Setpoint[1] = 0; Setpoint[2] = -1.5; // Frame is defined: x (+), y (-), z (-) 
        //     // std::cout << " Still Hovering " << std::endl;
        //     if (timeSinceLastRequest >= holdTime){
        //         stage = 1;
        //         std::cout << "Stage: " << stage << std::endl;
        //         lastRequestSimTime = currentSimTime;
        //     }
        //     break;
        // case 1:
        //     Setpoint[0] = 1; Setpoint[1] = -1; Setpoint[2] = -2;
        //     break;
        // case 2:
        //     Setpoint[0] = -2; Setpoint[1] = -1; Setpoint[2] = -4;
        //     break;
        // case 3:
        //     Setpoint[0] = 1; Setpoint[1] = 2; Setpoint[2] = -3;
        //     break;
        // case 4:
        //     Setpoint[0] = 1.5*std::sin(2.0*3.1415926535897931*timeSinceLastRequest/16.0);
        //     Setpoint[1] = 0.75*std::sin(4.0*3.1415926535897931*timeSinceLastRequest/16.0);
        //     Setpoint[2] = -0.5 + 0.5*std::sin(2.0*3.1415926535897931*timeSinceLastRequest/16.0);
        //     break;
        // case 5:
        //     Setpoint[0] = 0; Setpoint[1] = 0; Setpoint[2] = -1.5;
        //     break;            
        // default:
        //     break;
        // }
            
        // if (stage > 0 && stage != 4 && getDistance(Setpoint, currLoc) < 0.2 && timeSinceLastRequest > 15.0){
        //     stage += 1;
        //     std::cout << "Stage: " << stage << std::endl;
        //     // Update Request Time
        //     lastRequestSimTime = currentSimTime;
        // }

        // if (stage != 4){
        //     StabController(dv, Kv12, Param, Setpoint, target_force);
        //     // IntrgralStabController(dv, Kv15, Param, Setpoint, target_force, err, timeSinceLastRequest);
        // }
        
        // if (stage == 4){
        //     if ( timeSinceLastRequest > 32.0){
        //         std::cout << "Tracking Done! .." << std::endl;
        //         stage += 1;
        //         lastRequestSimTime = currentSimTime;
        //     }
        //     else{
        //         TracController(dv, Kv12, Param, timeSinceLastRequest, target_force);
        //         //std::cout << "Still Tracking" << std::endl;
        //     }
        // }


    }

    void checkMissionStage(double mission_time_span) {
        if(ros::Time::now().toSec() - mission_last_called_.toSec() >= mission_time_span) {
            mission_last_called_ = ros::Time::now();
            mission_stage_ += 1;
            ROS_INFO_STREAM("[exeMission] Stage " << mission_stage_-1 << " ended, switching to stage " << mission_stage_);
        }
    }

    ~SlsForceControlPlugin()
    {
        // if (logTxtFile.is_open())
        // {
        //     logTxtFile.close();
        // }
    }
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SlsForceControlPlugin)
} 