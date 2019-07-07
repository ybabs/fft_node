#ifndef FFT_NODE_H
#define FFT_NODE_H
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
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

const float FFT_RESOLUTION = 292.2695;


class STM32Process
{

    public:
        STM32Process();
        template <typename T>
        cv::Mat plotFFTPoints(std::vector<T>& vals, int YRange[2]);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void processSerialData();
        void publishFFTplot();


    private:
        std::vector<unsigned char> data_vector;
        std_msgs::Float32MultiArray converted_values;
        serial_processing::fft msg;
        serial::Serial ser;
        image_transport::Publisher image_pub;
        ros::Publisher fft_points_pub;
        ros::NodeHandle nh;
        //image_transport::ImageTransport transport;
        cv::Mat graph;


};


#endif