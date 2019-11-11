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
#include "sensor_msgs/ChannelFloat32.h"

#define SERIAL_LENGTH 2052
#define FLOAT_SIZE 1024
#define FFT_SIZE 512
 
const float FFT_RESOLUTION = 500000/1024; // Sampling frequency / fft length  


class STM32Process
{

    public:
    // Constructor
        STM32Process(const ros::NodeHandle &node_handle);
        ~STM32Process() = default;
        void init();
        void processSerialData();
        std::vector<float> getPlotData();
        int* getYRange();

    private:

        // public node handle
        ros::NodeHandle nh;
        // private node handle
        ros::NodeHandle pnh;



        std::vector<unsigned char> data_vector;
        std_msgs::Float32MultiArray converted_values;
        serial_processing::fft msg;
        serial::Serial ser;
        ros::Publisher fft_points_pub;
        int process_time;
        int sequence_number;

        std::thread thread_;
        // For Plot
        std::vector<float> plot_vals;
        int y_range[2];


};


#endif