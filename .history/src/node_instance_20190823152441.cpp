#include "fft_process/fft_node.hpp"
#include <boost/thread>

int main(int argc, char** argv)
{
     std::string node_name = "fft_proc";
    ros::init(argc, argv, "node_name");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    ros::AsyncSpinner s(4);

    STM32Process process(nh);

    ROS_INFO("Running Thread in async multi-threaded mode");

   

    s.start();

    ros::waitForShutdown();

    //return 0;
    
   
}