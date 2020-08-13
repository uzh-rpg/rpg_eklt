
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include "synchronizer.h"


namespace synchronizer
{

Synchronizer::Synchronizer(std::string input_video_path,
                           std::string input_bag_path,
                           std::string image_topic,
                           std::string trigger_topic,
                           std::string input_image_topic )
                           : init_(false), images_from_topic_(false)
{
    // input bag open
    rosbag::Bag input_bag;
    try
    {
        input_bag.open(input_bag_path, rosbag::bagmode::Read);
    }
    catch(rosbag::BagIOException e)
    {
        std::cerr << "Error: could not open rosbag: " << input_bag_path << std::endl;
    }
    
    // video capture 
    cv::VideoCapture video_handle(input_video_path);
    if (!input_video_path.empty())
    {
        CHECK(video_handle.isOpened()) << "Cannot open the video file " << input_video_path << std::endl;
    }
    else
    {
        images_from_topic_ = true;
    }

    // output bag
    rosbag::Bag output_bag;
    std::string path_to_output_rosbag = input_bag_path + ".synchronized";   
    output_bag.open(path_to_output_rosbag, rosbag::bagmode::Write);

    rosbag::View view(input_bag);    

    const uint32_t num_messages = view.size();
    uint32_t message_index = 0;

    rosbag::View img_view(input_bag, rosbag::TopicQuery({input_image_topic}));

    for (rosbag::MessageInstance const m : img_view)
    {
        cv_bridge::CvImagePtr cv_ptr; 
        sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8); 
        img_buffer_.push_back(cv_ptr->image.clone());
    }

    for (rosbag::MessageInstance const m : view)
    {
        ros::Time message_time = m.getTime();

        if (m.getTopic() == trigger_topic)
        {
            CHECK(m.getDataType() == "prophesee_event_msgs/Event") << "Trigger Topic message must be of type prophesee_event_msgs/EventArray and is " << m.getDataType();

            trigger_events_.push_back(*(m.instantiate<Event>()));

            message_time = trigger_events_[trigger_events_.size()-1].ts;
        }
        
        // not trigger events
        if (m.getDataType() == "prophesee_event_msgs/EventArray" && m.getTopic() != trigger_topic)
        {
            EventArrayConstPtr event_array = m.instantiate<EventArray>();
            if (!init_)
            {
                width_ = event_array->width;
                height_ = event_array->height;
                init_ = true;
            }

            for (Event e : event_array->events)
            {
                events_.push_back(e);
            }

            message_time = events_[events_.size()-1].ts;
        }


        
        if (events_.size()>0)
        {

            processTriggers(video_handle, 
                            output_bag,
                            image_topic,
                            input_image_topic);
        }
        if(m.getDataType() == "sensor_msgs/Imu")
        {
            sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            output_bag.write(m.getTopic(), imu_msg->header.stamp, m);
        }
        else if (m.getDataType() == "sensor_msgs/Image")
        {
            if ( m.getTopic() == input_image_topic)
                continue;

            output_bag.write(m.getTopic(), m.instantiate<sensor_msgs::Image>()->header.stamp, m);
        }
        else if (m.getDataType() == "prophesee_event_msgs/Event")
        {
            output_bag.write(m.getTopic(), m.instantiate<Event>()->ts, m);
        }
        else
        {
            output_bag.write(m.getTopic(), m.getTime(), m);
        }

    }

    output_bag.close();
    input_bag.close();
}


void Synchronizer::processTriggers(cv::VideoCapture& video_handle,
                                   rosbag::Bag& output_bag,
                                   std::string image_topic,
                                   std::string input_image_topic)
{   
    if (trigger_events_.size()==0 || events_.size() == 0)
        return; 

    static int trigger_counter = 0;
    static auto first_trigger_t = trigger_events_[0].ts; 
    auto last_trigger_t = trigger_events_[0].ts; 
    VLOG(2) << "----";
    VLOG(2) << "Processing triggers " << trigger_events_.size() << " t0=" << trigger_events_[0].ts.toNSec()-47000000000 - first_trigger_t.toNSec() << " t1=" << trigger_events_[trigger_events_.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec();
    VLOG(2) << "Processing events " << events_.size() << " t0=" << events_[0].ts.toNSec()-47000000000 - first_trigger_t.toNSec() << " t1=" << events_[events_.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec();
    //if (events_.size()>0)
    //    VLOG(2) << "\tevents: t0=" << events_[0].ts.toNSec()-first_trigger_t.toNSec()-47000000000 << " t1=" << events_[events_.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec() ;
    //if (trigger_events_.size()>0)
    //    VLOG(2) << "\ttriggs: t0=" << last_trigger_t.toNSec()-47000000000-first_trigger_t.toNSec() << " t1=" << trigger_event.ts.toNSec()-47000000000-first_trigger_t.toNSec() ;
    
    while (trigger_events_.size()>0 && events_.size()>0 &&(trigger_events_[0].ts < events_[events_.size()-1].ts))
    {
        VLOG(2) << "\t----Batch";
        VLOG(2) << "\tProcessing triggers " << trigger_events_.size() << " t0=" << trigger_events_[0].ts.toNSec()-47000000000 - first_trigger_t.toNSec() << " t1=" << trigger_events_[trigger_events_.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec();
        VLOG(2) << "\tProcessing events " << events_.size() << " t0=" << events_[0].ts.toNSec()-47000000000 - first_trigger_t.toNSec() << " t1=" << events_[events_.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec();
        


        Event trigger_event = trigger_events_[0];
        std::vector<Event> event_vector;

        //VLOG(1) << "Processing:";
        //VLOG(1) << "\tevents: t0="<<events_[0].ts.toNSec()-47000000000 - first_trigger_t.toNSec() << " t1=" << events_[events_.size()-1].ts.toNSec()-47000000000 - first_trigger_t.toNSec(); 
        //VLOG(1) << "\ttriggers: " << trigger_events_.size() << " t0="<<trigger_events_[0].ts.toNSec()-47000000000 - first_trigger_t.toNSec() << " t1=" << trigger_events_[trigger_events_.size()-1].ts.toNSec()-47000000000 - first_trigger_t.toNSec(); 

        //VLOG(2) << "Processing trigger " << trigger_counter++ << " t_trigger=" << trigger_event.ts.toNSec()-47000000000 - first_trigger_t.toNSec();
        //if (events_.size()>0)
        //    VLOG(2) << "\tevents: t0=" << events_[0].ts.toNSec()-first_trigger_t.toNSec()-47000000000 << " t1=" << events_[events_.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec() ;
        //if (trigger_events_.size()>0)
        //    VLOG(2) << "\ttriggs: t0=" << last_trigger_t.toNSec()-47000000000-first_trigger_t.toNSec() << " t1=" << trigger_event.ts.toNSec()-47000000000-first_trigger_t.toNSec() ;

        while (events_.size()>0 && events_[0].ts < trigger_event.ts)
        {
            Event e = events_[0];
            event_vector.push_back(e);
            events_.pop_front();
        }

        
        //if (event_vector.size()>0)
        //    VLOG(2) << "\tvectors: t0=" << event_vector[0].ts.toNSec()-47000000000-first_trigger_t.toNSec() << " t1=" << event_vector[event_vector.size()-1].ts.toNSec()-47000000000-first_trigger_t.toNSec() ;
        //
        //
        //VLOG(2) << "\tPackaged " << event_vector.size() << " events."; 
        
        last_trigger_t = trigger_event.ts;
        trigger_events_.pop_front();

        EventArray event_msg;
        event_msg.events = event_vector;
        event_msg.width = width_;
        event_msg.height = height_;

        if (event_vector.size()>0)
        {
            event_msg.header.stamp = event_vector[event_vector.size()-1].ts;
        }
        else 
        {
            event_msg.header.stamp = trigger_event.ts;
        }

        std::string topic;
        if (trigger_event.polarity)
        {
            topic = "/prophesee/camera/inter_frame"; 
        }
        else
        {
            topic = "/prophesee/camera/intra_frame";

            cv::Mat img;
            //success = !images_from_topic_ && video_handle.read(img)

            if (!img_buffer_.empty())
            {
                img = img_buffer_.front().clone();
                img_buffer_.pop_front();

                // helper for publishing images
                static cv_bridge::CvImage cv_image;

                cv_image.encoding = "bgr8";
                cv_image.image = img.clone();
                cv_image.header.stamp = event_msg.header.stamp;
                VLOG(1) << "Timestamp of image " << event_msg.header.stamp.toNSec();
                output_bag.write(image_topic, event_msg.header.stamp, cv_image);
            }
        }

        output_bag.write(topic, event_msg.header.stamp, event_msg);
    }    
}

};
