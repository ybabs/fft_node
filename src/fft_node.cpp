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
using namespace std;


std::vector<unsigned char> data_vector;
std_msgs::Float32MultiArray converted_values;
serial_processing::fft msg;
serial::Serial ser;
const float FFT_RESOLUTION = 292.2695;
void write_callback(const serial_processing::fft::ConstPtr& array);
 image_transport::Publisher image_pub_;
 cv::Mat graph;


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

template <typename T>
cv::Mat plotGraph(std::vector<T>& vals, int YRange[2])
{

    auto it = minmax_element(vals.begin(), vals.end());
    float scale = 1./ceil(*it.second - *it.first); 
    float bias = *it.first;
    int rows = YRange[1] - YRange[0] + 1;
    cv::Mat image = cv::Mat::zeros( rows, vals.size(), CV_8UC3 );
    image.setTo(0);
    for (int i = 0; i < (int)vals.size()-1; i++)
    {
        cv::line(image, cv::Point(i, rows - 1 - (vals[i] - bias)*scale*YRange[1]), cv::Point(i+1, rows - 1 - (vals[i+1] - bias)*scale*YRange[1]), cv::Scalar(255, 0, 0), 1);
    }

    // cv::namedWindow("image", CV_WINDOW_NORMAL);
    // cv::resizeWindow("image", 1024,1024);
    // cv::imshow("image", image);   
    // //cv::waitKey();   
    // cv::destroyWindow("image"); 

 
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->image = image;
    image_pub_.publish(cv_ptr->toImageMsg());

    ROS_INFO("Image published");
    return image;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  graph = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::imshow( "image", graph );
  cv::waitKey(30);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fft_proc");
    
    ros::NodeHandle nh;

    ros::Publisher read_pub = nh.advertise<serial_processing::fft>("FFT", 512);
    image_transport::ImageTransport it_(nh);
    image_pub_ = it_.advertise("/traj_output", 1);
    ros::Subscriber array_sub = nh.subscribe("FFT", 512, write_callback); 

     ros::Subscriber sub = nh.subscribe("/traj_output", 1000, imageCallback);
   

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

                   // ADD GPS Timestamped Messages here.....


                   // Show graph here..
                   int range[2] = {0, float_vector.size()};
                   graph = plotGraph(float_vector, range);
                   //cv::imshow( "image", graph );


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
    
};