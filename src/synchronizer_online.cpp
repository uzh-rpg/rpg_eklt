#include <sensor_msgs/Image.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <cv_bridge/cv_bridge.h>

#include "synchronizer_online.h"
#include <thread>

namespace synchronizer
{

SynchronizerOnline::SynchronizerOnline(ros::NodeHandle &nh,
                                       std::string image_topic,
                                       std::string trigger_topic,
                                       std::string event_topic)
                                       : nh_(nh), it_(nh)
{
    img_sub_ = it_.subscribe(image_topic, 0, &SynchronizerOnline::imageCallback, this);
    trigger_sub_ = nh_.subscribe(trigger_topic, 0, &SynchronizerOnline::triggerCallback, this);
    img_pub_ = it_.advertise(image_topic+"_sync", 0);

    std::thread worker(&SynchronizerOnline::run, this);
    worker.detach();
}

void SynchronizerOnline::run()
{
    ros::Rate rate(200);
    while (ros::ok())
    {
        rate.sleep();
        processBuffers();
    }
}

SynchronizerOnline::~SynchronizerOnline()
{
    img_pub_.shutdown();
}

void SynchronizerOnline::triggerCallback(prophesee_event_msgs::Event trigger)
{
    std::unique_lock<std::mutex> lck(mutex_);
    trigger_buffer_.push_back(trigger);
}

void SynchronizerOnline::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    std::unique_lock<std::mutex> lck(mutex_);
    img_buffer_.push_back(*msg);
}


void SynchronizerOnline::processBuffers()
{
    if (img_buffer_.size() == 0 || trigger_buffer_.size() < 2)
        return;
    
    while (!(img_buffer_.empty() ||  trigger_buffer_.size() < 2))
    {
        auto trig_first = trigger_buffer_[0];
        auto trig_second = trigger_buffer_[1];
        auto img = img_buffer_[0];

        CHECK(trig_first.polarity && !trig_second.polarity) << "Polarities of triggers are not correct.";
        
        ros::Time img_stamp((trig_first.ts.toSec()+trig_second.ts.toSec())/2);

        img.header.stamp = img_stamp;
        img_pub_.publish(img);        

        VLOG(1) << "Publishing image! Num triggers is: " << trigger_buffer_.size() << " " << img_buffer_.size();

        img_buffer_.pop_front();
        trigger_buffer_.pop_front();
        trigger_buffer_.pop_front();
    }
}

};
