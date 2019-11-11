#ifndef FFT_NODE_HPP
#define FFT_NODE_HPP
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <thread>
#include <array>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "serial_processing/fft.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/ChannelFloat32.h"
#include <image_transport/image_transport.h>

#define SERIAL_LENGTH 2052
#define FLOAT_SIZE 1024
#define FFT_SIZE 512
 
const float FFT_RESOLUTION = 500000/1024; // Sampling frequency / fft length  


class STM32Process
{

    public:
    // Constructor
        STM32Process(const ros::NodeHandle &node_handle, 
        const ros::NodeHandle &private_node_handle);

        ~STM32Process() = default;

        /**
         * Initialise Publishers, subscribers and periodic timers
         * 
         */
        void init();
        /** Subscriber callbacks 
         * Deal with publishing the FFT image in this callback 
         * @param event
         * 
         */
           void imageCallback(const ros::TimerEvent& event);

        /*
         * 
         */
        void processSerialData();

        template <typename T>
        cv::Mat plotFFTPoints(std::vector<T>& vals, int YRange[2]);


    private:

        // public node handle
        ros::NodeHandle nh;
        // private node handle
        ros::NodeHandle pnh;

         // periodic timer
         ros::Timer periodic_timer;

        std::vector<unsigned char> data_vector;
        std_msgs::Float32MultiArray converted_values;
        serial_processing::fft msg;
        serial::Serial ser;
        image_transport::Publisher image_pub;
        ros::Publisher fft_points_pub;
        cv::Mat graph;
        int process_time;
        int sequence_number;

        std::thread thread_;




        // For Plot
        std::vector<float> plot_vals;
        int y_range[2];


};


#endif