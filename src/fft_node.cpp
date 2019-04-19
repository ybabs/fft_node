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
float converted_values[512];
//unsigned char data[2053];
//std_msgs::Float32MultiArray converted_values;

serial_processing::fft msg;
unsigned char temp_buffer[4];
int len ;
serial::Serial ser;



float fft_vals[1023];
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

    ros::Publisher read_pub = nh.advertise<serial_processing::fft>("FFT", 1024);
    ros::Subscriber array_sub = nh.subscribe("FFT", 1024, write_callback); 

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
                    
                    // for(std::vector<unsigned char>::const_iterator i = bytes.begin(); i != bytes.end(); ++i)
                    // {
                    //     std::cout << *i <<endl;
                    // }

                    // unsigned char* byte = &(bytes[0]);
                    // vector<float>floatValue;

                    //std::cout << bytes.size() << std::endl;
                     
                    
                    float float_values[513];
                    //unsigned char temp_buffer[4];


                     memcpy(&float_values[0], &bytes[0], bytes.size());

                    std::vector<float>float_vector{std::begin(float_values), std::end(float_values)};
                    // float float_values[513] = { 0 };
                    // int j = 0;
                    // for(size_t i = 4; i < 2052; i+=4)
                    // {

                    //     temp_buffer[0] = bytes[i - 4];
                    //     temp_buffer[1] = bytes[i - 3];
                    //     temp_buffer[2] = bytes[i - 2];
                    //     temp_buffer[3] = bytes[i - 1];

                    //     memcpy(&float_values[j], temp_buffer, sizeof(float));
                    //     //memcpy(float_values[j], temp_buffer, sizeof(float));

                    //     //float_values.insert(float_values.end(), &temp_buffer[0], &temp_buffer[4]);

                    //     j++;
                    // remove last element from the vector
                    float_vector.erase(float_vector.end()-1);
                        
                    std::cout << float_vector.size() << std::endl;


                    // }


                    
                    for(int i = 60; i < 70; i++)
                    {
                    
                         std::cout << i << "---------" << float_vector[i] << endl;
                         
                    }

                     std::cout << "EN--------D" << endl;



                   
                
                    //float_values.clear();
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