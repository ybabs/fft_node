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

#define SERIAL_LENGTH 2052
#define FLOAT_SIZE 1024
using namespace std;


std::vector<unsigned char> data_vector;
std_msgs::Float32MultiArray converted_values;
serial_processing::fft msg;
serial::Serial ser;
const float FFT_RESOLUTION = 292.2695;
void write_callback(const serial_processing::fft::ConstPtr& array);

void write_callback(const serial_processing::fft::ConstPtr& array)
{
    // int i = 0;

    //          for(vector<float>::iterator it = array.data.begin(); it !=converted_values.data.end(); ++it)
    //          {
    //              //read_pub.publish(*it);
    //              fft_vals[i] = *it;
    //              i++;

    //          }

    // return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fft_proc");
    
    ros::NodeHandle nh;

    ros::Publisher read_pub = nh.advertise<serial_processing::fft>("FFT", 512);
    ros::Subscriber array_sub = nh.subscribe("FFT", 512, write_callback); 

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout time_out = serial::Timeout(100,100,0,100,0);
        ser.setTimeout(time_out);
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialised");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(50);


     float result;
     int j = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        if(ser.available())
        {


        size_t data_available = ser.available();

        if(data_available > 2052)
        {
          ROS_WARN_STREAM("Serial read buffer is " << data_available << ", now flushing in an attempt to catch up.");
          ser.flushInput();
        }

        std::string stop_string;
        ser.readline(stop_string, 2052, "stop");

        // check what the string ends with
             if(boost::algorithm::ends_with(stop_string, "stop"))
             {

                 //ROS_INFO("Boost Error");

                  unsigned int num = 0;

                if(stop_string.length() == 2052)
                {
                    
                    std::vector<unsigned char>bytes(stop_string.begin(), stop_string.end());
                                                        
                    float float_values[513];
                    
                    memcpy(&float_values[0], &bytes[0], bytes.size());

                    std::vector<float>float_vector{std::begin(float_values), std::end(float_values)};
                    
                    // remove last element from the vector (STOP)
                    float_vector.erase(float_vector.end()-1);
                        
                    //std::cout << float_vector.size() << std::endl;
               
                   msg.header.stamp = ros::Time::now();

                   for(std::vector<float>::iterator i = float_vector.begin(); i != float_vector.end(); ++i)
                   {
                       msg.fftAmplitude.data.push_back(*i);
                   }

                    read_pub.publish(msg);
                    int max_index = max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end())- msg.fftAmplitude.data.begin();
                    float max = *max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end());
                    float frequency = (max_index * FFT_RESOLUTION)/1000;
                    ROS_INFO("%f kHz Frequency at Index %d with amplitude %f", frequency , max_index, max);
                    float_vector.clear();
                    msg.fftAmplitude.data.clear();
                }

             }

             else if(!boost::algorithm::ends_with(stop_string, "stop"))
             {
                 ROS_WARN("Junk ByteStream");
                 ser.flushInput();
             }        
                      
                

        }

        loop_rate.sleep();
    }
    
}