
#include "fft_process/fft_image.hpp"



void STM32Process::imageCallback(const ros::TimerEvent &event)
{
    // auto it = std::minmax_element(plot_vals.begin(), plot_vals.end()); // find the min and max element in the vector
    // float scale = 1./ceil(*it.second - *it.first);
    // float bias = *it.first;
    // int rows = y_range[1] - y_range[0] + 1;       // number of elements

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

   // processSerialData();

}