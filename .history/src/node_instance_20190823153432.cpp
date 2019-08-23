#include "fft_process/fft_node.hpp"
#include <boost/thread.hpp>

int main(int argc, char** argv)
{
     std::string node_name = "fft_proc";
    ros::init(argc, argv, "node_name");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    ros::AsyncSpinner s(4);

    STM32Process process(nh);

    //ros::Rate loop_rate(1000);

    //while(ros::ok())
   // {
        //     ros::spinOnce();

        //     loop_rate.sleep();
   // }

   // ROS_INFO_STREAM("Running Thread in async multi-threaded mode" << boost::this_thread::get_id());

   

    s.start();

    ros::waitForShutdown();

    return 0;
    
   
}