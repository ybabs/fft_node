#include "fft_process/fft_node.hpp"

int main(int argc, char** argv)
{
     std::string node_name = "fft_proc";
    ros::init(argc, argv, "node_name");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    STM32Process process(nh, nh_private);

    ROS_INFO("Running Thread in async multi-threaded mode");

    ros::AsyncSpinner s(4);

    s.start();

    ros::waitForShutdown();

    return 0;
    
   
}