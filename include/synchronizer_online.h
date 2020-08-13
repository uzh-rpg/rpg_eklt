#pragma once

#include <string>
#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>


namespace synchronizer
{
class SynchronizerOnline
{
public:
    SynchronizerOnline(ros::NodeHandle &nh, std::string image_topic, std::string trigger_topic, std::string event_topic);
    ~SynchronizerOnline();

    void triggerCallback(prophesee_event_msgs::Event trigger);
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
    void processBuffers();
    void run();

private:
    std::deque<prophesee_event_msgs::Event> trigger_buffer_;
    std::deque<sensor_msgs::Image> img_buffer_;

    ros::NodeHandle nh_;

    ros::Subscriber trigger_sub_;
    image_transport::Subscriber img_sub_;
    image_transport::Publisher img_pub_;    
    image_transport::ImageTransport it_;
    std::mutex mutex_;
};
} // synchronizer
