#include "fft_process/fft_node.h"
#include <fstream>



bool STM32Process::isFFTReady()
{
    return fftComputeFlag;
}

void STM32Process::setupPort()
 {
	 try
    {
        ser.setPort("/dev/ttyAMA1");
        if(dev_id == F7_DEVICE)
        {
          ser.setBaudrate(F7_BAUD);
        }
        else if(dev_id == H7_DEVICE)
        {
            ser.setBaudrate(H7_BAUD);
        }
        serial::Timeout time_out = serial::Timeout(100,100,0,100,0);
        ser.setTimeout(time_out);
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port: %s", e.what());
    }
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialised");
    }
    else
    {
        ROS_ERROR("An error occured");
    }
 }


STM32Process::STM32Process()
{
    fft_points_pub = nh.advertise<porpdaq::fft>("FFT", FFT_SIZE);
    record_subscriber = nh.subscribe("/uav_agent/record", 10, &STM32Process::serialCallback, this);
    image_transport::ImageTransport transport(nh);
    image_pub = transport.advertise("/fft_plot", 1);

    if(ros::param::has("/dev_id"))
    {
        ros::param::get("/dev_id", dev_id);
    }
    
    if(ros::param::has("/dev_id"))
    {   
        ros::param::get("/port", port);
    }

    ROS_INFO("Device is a %d type on %s", dev_id, port);
    setupPort();
    this->timer = nh.createTimer(ros::Duration(1.0/30.0), &STM32Process::processDataCallback, this); 
}

void STM32Process::writeStartData()
{
	ROS_INFO("Recording started");
	start_data[0] = 0x31;
	ser.write(start_data, 1);
}	

void STM32Process::writeStopData()
{
	ROS_INFO("Recording stopped");
	stop_data[0]= 0x32;
	
	ser.write(stop_data, 1);
	
}

void STM32Process::serialCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    int rec_cmd = msg->data;
    ROS_INFO("Callback");

    switch(rec_cmd)
    {
        case 1:
            writeStartData();
            break;
        case 2: 
            writeStopData();
            break;
        default:
            break;
    }

}

template <typename T>
cv::Mat STM32Process::plotFFTPoints(std::vector<T>& vals, int y_range[2])
{
    auto it = std::minmax_element(vals.begin(), vals.end()); // find the min and max element in the vector
    float scale = 1./ceil(*it.second - *it.first);
    float bias = *it.first;
    int rows = y_range[1] - y_range[0] + 1;       // number of elements


    cv::Mat fft_plot = cv::Mat::zeros(rows, vals.size(), CV_8UC3);

    fft_plot.setTo(0);

    // draw line
    for(int i = 0; i < (int)vals.size()- 1; i++ )
    {
        cv::line(fft_plot, cv::Point(i, rows - 1 - (vals[i] - bias)*scale*y_range[1]), 
                           cv::Point(i+1, rows - 1 - (vals[i+1] - bias)*scale*y_range[1]), 
                           cv::Scalar(255, 0, 0), 1);
    }

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->image = fft_plot;
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    image_pub.publish(cv_ptr->toImageMsg());

    //ROS_INFO("Image published");
    return fft_plot;  // No need for this bruh...

}

void STM32Process::processDataCallback(const ros::TimerEvent&)
{
    //ROS_INFO("Callback Serial");
    float result;
    int j = 0; 
     
         if(ser.available())
         {
             size_t data_available = ser.available();

             if(data_available > 2052)
             {
                ROS_WARN_STREAM("Serial read buffer is " << data_available << ", now flushing in an attempt to catch up.");
                ser.flushInput();
             }

            std::string delimiter_string;
            currTime = ros::Time::now();
            ser.readline(delimiter_string, 2052, "stop");
            ros::Duration time = currTime - prevTime;
            prevTime = currTime;

             // check what the string ends with
             if(boost::algorithm::ends_with(delimiter_string, "stop"))
             {
                 unsigned int num = 0;

                 if(delimiter_string.length() == 2052) 
                 {
                      
                     std::vector<unsigned char>uart_bytes(delimiter_string.begin(), delimiter_string.end());

                     float uart_float_values[513];

                     memcpy(&uart_float_values[0], &uart_bytes[0], uart_bytes.size());

                      std::vector<float>float_vector{std::begin(uart_float_values), std::end(uart_float_values)};

                    // remove last element from the vector (STOP)
                    float_vector.erase(float_vector.end()-1);


                    msg.header.stamp = ros::Time::now();

                    for(std::vector<float>::iterator i = float_vector.begin(); i != float_vector.end(); ++i)
                    {
                        msg.fftAmplitude.data.push_back(*i);
                    }
             

                     double dt = time.toSec();

                     // Show graph here..
                    // ROS_INFO("Dt %f", dt);
                   //int range[2] = {0, (int)float_vector.size()};
                   //imagePlot = plotFFTPoints(float_vector, range);  // change plot_graph to a void()
                    // publish FFT Messages     
                    fft_points_pub.publish(msg);
                    // Find index where max value is stored
                    int max_index = max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end())- msg.fftAmplitude.data.begin();
                    // computes the value of the highest amplitude
                    float max = *max_element(msg.fftAmplitude.data.begin(), msg.fftAmplitude.data.end());
                    float frequency = 0;
                    // return frequency
                    if(dev_id == F7_DEVICE)
                    {
                        frequency = (max_index * F7_FFT_RESOLUTION)/1000;
                    }

                    else if(dev_id == H7_DEVICE)
                    {
                        frequency = (max_index * H7_FFT_RESOLUTION)/1000;
                    }                    
                   
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fft_proc");


    STM32Process process;
    
    ros::spin();
   
    return 0;
    
   
}
