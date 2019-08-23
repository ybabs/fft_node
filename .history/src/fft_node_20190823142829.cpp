#include "fft_process/fft_node.hpp"
#include <fstream>

STM32Process::STM32Process(const ros::NodeHandle &node_handle, 
        const ros::NodeHandle &private_node_handle):
        nh(node_handle),
        pnh(private_node_handle),
        plot_vals(0),
        y_range{}

{
        this->init();
}


void STM32Process::init()
{
    fft_points_pub = pnh.advertise<serial_processing::fft>("FFT", FFT_SIZE);
    image_transport::ImageTransport transport(pnh);
    image_pub = transport.advertise("/fft_plot", 1);

    periodic_timer = pnh.createTimer(ros::Duration(0.01), &STM32Process::imageCallback, this);

    processSerialData();

}


void STM32Process::imageCallback(const ros::TimerEvent &event)
{
    auto it = std::minmax_element(plot_vals.begin(), plot_vals.end()); // find the min and max element in the vector
    float scale = 1./ceil(*it.second - *it.first);
    float bias = *it.first;
    int rows = y_range[1] - y_range[0] + 1;       // number of elements

    // cv::Mat fft_plot = cv::Mat::zeros(rows, plot_vals.size(), CV_8UC3);
    // fft_plot.setTo(0);

    // // draw line
    // for(int i = 0; i < (int)plot_vals.size()- 1; i++ )
    // {
    //     cv::line(fft_plot, cv::Point(i, rows - 1 - (plot_vals[i] - bias)*scale*y_range[1]), 
    //                        cv::Point(i+1, rows - 1 - (plot_vals[i+1] - bias)*scale*y_range[1]), 
    //                        cv::Scalar(255, 0, 0), 1);
    // }

    // cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    // cv_ptr->image = fft_plot;
    // cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    // image_pub.publish(cv_ptr->toImageMsg());

}

void STM32Process::processSerialData()
{

    float result;
    int j = 0;
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
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialised");
    }
    else
    {
        ROS_ERROR_STREAM("An error occured");
    }  

         if(ser.available())
         {
              size_t data_available = ser.available();
            // int data_available = ser.available();
            // ROS_INFO("data %d", data_available );

             if(data_available > 2052)
             {
                ROS_WARN_STREAM("Serial read buffer is " << data_available << ", now flushing in an attempt to catch up.");
                ser.flushInput();
             }

             std::string delimiter_string;
            // ser.readline(delimiter_string, 2060, "stop");
            //curr = ros::Time::now();
            ser.readline(delimiter_string, 2052, "stop");
            //ros::Duration time = curr - prev;
            // prev = curr;

             // check what the string ends with
             if(boost::algorithm::ends_with(delimiter_string, "stop"))
             {
                 unsigned int num = 0;

                 if(delimiter_string.length() == 2052) //2060
                 {
                      
                     std::vector<unsigned char>uart_bytes(delimiter_string.begin(), delimiter_string.end());

                     //float uart_float_values[515];
                     float uart_float_values[513];

                     memcpy(&uart_float_values[0], &uart_bytes[0], uart_bytes.size());

                      std::vector<float>float_vector{std::begin(uart_float_values), std::end(uart_float_values)};

                    // remove last element from the vector (STOP)
                    float_vector.erase(float_vector.end()-1);
                    //sequence_number =  float_vector[0];
                   // process_time = float_vector[1];

                    //remove first two elements from the vector 
                   // float_vector.erase(float_vector.begin(), float_vector.begin()+1);

                    msg.header.stamp = ros::Time::now();

                    for(std::vector<float>::iterator i = float_vector.begin(); i != float_vector.end(); ++i)
                    {
                        msg.fftAmplitude.data.push_back(*i);
                    }

                    y_range[0] = 0;
                    y_range[1] =  (int)float_vector.size();
                    plot_vals =float_vector ; // change plot_graph to a void()

                    // publish FFT Messages
                    fft_points_pub.publish(msg);
                    // Find index where max value is stored
                    int max_index = max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end())- msg.fftAmplitude.data.begin();
                    // computes the value of the highest amplitude
                    float max = *max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end());
                    // return frequency
                    float frequency = (max_index * FFT_RESOLUTION)/1000;
                    // ROS_INFO("Packet Number: %d, Process time %d us, %f kHz Frequency at Index %d with amplitude %f", frequency , max_index, max);
                    ROS_INFO("%f kHz Frequency at Index %d with amplitude %f", frequency , max_index, max);

                    float_vector.clear();
                    msg.fftAmplitude.data.clear();
                 }
             }

             else if(!boost::algorithm::ends_with(delimiter_string, "stop"))
             {
                 ROS_WARN("Junk ByteStream");
                 ser.flushInput();
             }             

         }

}


