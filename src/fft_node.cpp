#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "serial_processing/fft.h"

#define SERIAL_LENGTH 4096
#define FLOAT_SIZE 1024

using namespace std;

//std::vector<unsigned char> data;
float converted_values[512];
unsigned char data[2053];
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

           //len = ser.read(data, SERIAL_LENGTH);

        size_t data_available = ser.available();

        if(data_available > 2053)
        {
          ROS_WARN_STREAM("Serial read buffer is " << data_available << ", now flushing in an attempt to catch up.");
          ser.flushInput();
        }

        uint8_t header_bytes[2];
        string msg = ser.readline(2053, "\n");
        //ser.read(data, 2053);

        std::cout << hex << (int)msg[0] << std::endl;

        // if(data[0] && data[1] == 0xFF)
        // {
        //     header_bytes[0] = data[2051];
        //     header_bytes[1] =  data[2052];
        //      ROS_INFO("Header packet %02x %02x", header_bytes[0], header_bytes[1]);
        // }

    //    std::cout<<"data" << hex << data << std::endl;
          

    //     //ser.flushInput();
    //       // cout << "len: "  << len << endl;
            
    //        for(int i = 7; i < 2053; i +=4)
    //        {
    //         temp_buffer[0] = data[i-4];
	// 	    temp_buffer[1] = data[i - 3];
	// 	    temp_buffer[2] = data[i - 2];
	// 	    temp_buffer[3] = data[i - 1];

	// 	    memcpy(&converted_values[j], temp_buffer, sizeof(float));
            
    //         //  std::copy(reinterpret_cast<const char*>(&temp_buffer[0]),
    //         //              reinterpret_cast<const char*>(&temp_buffer[4]),
    //         //             reinterpret_cast<char*>(&result));
    
       
    //       // msg.header.stamp = ros::Time::now();
    //       // msg.fftAmplitude.data.push_back(result);     
    //        // converted_values.data.push_back(result);

    //        std::cout << converted_values[j] << std::endl;
           
    //        j++;
    //     //    if(j >= 512)
    //     //    {
    //     //        j = 0;
               
    //     //        std::cout << *std::max_element(std::begin(converted_values), std::end(converted_values));
    //     //    }

    //        //std::cout << j << std::endl;
    //         }

           
             
    //        // float max = *max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end());
    //        // ROS_INFO("Length %f", max);
    //        // read_pub.publish(msg);                          
    //        // msg.fftAmplitude.data.clear();

        }

        loop_rate.sleep();
    }
    
}