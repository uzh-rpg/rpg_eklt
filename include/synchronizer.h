#pragma once

#include <string>
#include <deque>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>

using Event = prophesee_event_msgs::Event;
using EventArray = prophesee_event_msgs::EventArray;
using EventArrayConstPtr = prophesee_event_msgs::EventArrayConstPtr;



namespace synchronizer
{
class Synchronizer
{
public:
    Synchronizer(std::string input_video_path,
                 std::string input_bag,
                 std::string image_topic,
                 std::string trigger_topic,
                 std::string input_image_topic );
    
    void processTriggers(cv::VideoCapture& video_handle,
                         rosbag::Bag& output_bag,
                         std::string image_topic,
                         std::string input_image_topic);

private:
    std::deque<Event> trigger_events_;
    std::deque<Event> events_;
    std::deque<cv::Mat> img_buffer_;

    int height_;
    int width_;
    bool init_;
    bool images_from_topic_;
};
} // synchronizer
