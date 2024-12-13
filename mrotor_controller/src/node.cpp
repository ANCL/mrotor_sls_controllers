#include "mrotor_controller/mrotor_controller.hpp"
#include "mrotor_controller/sls_controller.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "mrotor_sls_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");   

    mrotorSlsCtrl* mrotorController = new mrotorSlsCtrl(nh, nh_private); 

    // The part below must be put after initializing the controller!
    dynamic_reconfigure::Server<mrotor_controller::MrotorControllerConfig> srv;
    dynamic_reconfigure::Server<mrotor_controller::MrotorControllerConfig>::CallbackType f;
    f = boost::bind(&mrotorCtrl::dynamicReconfigureCb, mrotorController, _1, _2);
    srv.setCallback(f);

    ros::spin();
    return 0;
}