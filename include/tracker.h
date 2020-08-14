#pragma once

#include <ros/ros.h>
#include <gflags/gflags.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include <image_transport/image_transport.h>

#include <deque>
#include <mutex>
#include <fstream>

#include "patch.h"
#include "optimizer.h"
#include "viewer.h"

DECLARE_double(tracking_quality);


namespace tracker
{
/**
 * @brief The Tracker class: uses Images to initialize corners and then tracks them using events.
 * Images are subscribed to and, when collected Harris Corners are extracted. Events falling in a patch around the corners
 * forming an event-frame-patch are used as observations and tracked versus the image gradient in the patch
 * at initialization.
 */
class Tracker
{
public:
    Tracker(ros::NodeHandle &nh, viewer::Viewer &viewer);
private:
    /**
   * @brief Initializes a viewer and optimizer with the first image. Also extracts first features.
   * @param image_it CV_8U gray-scale image on which features are extracted.
   */
    void init(const ImageBuffer::iterator& image_it);
    /**
     * @brief working thread
     */ 
    void processEvents();
    /**
     * @brief Blocks while there are no events in buffer
     * @param next event
    */ 
    inline void waitForEvent( dvs_msgs::Event& ev)
    {
        static ros::Rate r(100);

        while (true)
        {
            {
            std::unique_lock<std::mutex> lock(events_mutex_);
            if (events_.size()>0)
            {
                ev = events_.front();
                events_.pop_front();
                return;
            }
            }
            r.sleep();
            VLOG_EVERY_N(1, 30) << "Waiting for events.";
        }
    }
    /**
     * @brief blocks until first image is received
     */
    void waitForFirstImage(ImageBuffer::iterator& current_image_it);
    /**
    * @brief Always assigns image to the first image before time  t_start
    */
    inline bool updateFirstImageBeforeTime(ros::Time t_start, ImageBuffer::iterator& current_image_it)
    {
        bool next_image = false;
        auto next_image_it = current_image_it;

        while (next_image_it->first < t_start)
        {
            ++next_image_it;
            if (next_image_it == images_.end())
                break;

            if (next_image_it->first < t_start)
            {
                next_image = true;
                current_image_it = next_image_it;
            }
        }

        return next_image;
    }

    /**
     * @brief checks all features if they can be bootstrapped
     */
    void bootstrapAllPossiblePatches(Patches& patches, const ImageBuffer::iterator& image_it);
    /**
   * @brief bootstrapping features: Uses first two frames to initialize feature translation and optical flow.
   */
    void bootstrapFeatureKLT(Patch& patch, const cv::Mat& last_image, const cv::Mat& current_image);
    /**
     * @brief bootstrapping features: Uses first event frame to solve for the best optical flow, given 0 translation.
     */
    void bootstrapFeatureEvents(Patch& patch, const cv::Mat& event_frame);
    /**
     * @brief add new features
     */
    void addFeatures(std::vector<int>& lost_indices, const ImageBuffer::iterator& image_it);
    /**
     * @brief update a patch with the new event
     */
    void updatePatch(Patch& patch, const dvs_msgs::Event &event);
    /**
     * @brief reset patches that have been lost.
     */
    void resetPatches(Patches& new_patches, std::vector<int>& lost_indices, const ImageBuffer::iterator& image_it);
    
    /**
     * @brief initialize corners on an image
     */
    void initPatches(Patches& patches, std::vector<int>& lost_indices, const int& corners, const ImageBuffer::iterator& image_it);

    /**
     * @brief extract patches
     */
    void extractPatches(Patches &patches, const int& num_patches, const ImageBuffer::iterator& image_it);
    
    inline void padBorders(const cv::Mat& in, cv::Mat& out, int p)
    {
        out = cv::Mat(in.rows + p*2, in.cols + p*2, in.depth());
        cv::Mat gray(out, cv::Rect(p, p, in.cols, in.rows));
        copyMakeBorder(in, out, p, p, p, p, cv::BORDER_CONSTANT);
    }

    /**
     * @brief checks if the optimization cost is above 1.6 (as described in the paper)
     */
    inline bool shouldDiscard(Patch& patch)
    {
        bool out_of_fov = (patch.center_.y < 0 || patch.center_.y >= sensor_size_.height || patch.center_.x < 0 || patch.center_.x >= sensor_size_.width);
        bool exceeded_error = patch.tracking_quality_ < FLAGS_tracking_quality;

        return exceeded_error || out_of_fov;
    }
    
    /**
     * @brief sets the number of events to process adaptively according to equation (15) in the paper
     */
    void setBatchSize(Patch& patch, const cv::Mat& I_x, const cv::Mat& I_y, const double& d);
    /**
     * @brief ros callbacks for images and events
     */
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    /**
     * @brief Insert an event in the buffer while keeping the buffer sorted
     * This uses insertion sort as the events already come almost always sorted
     */
    inline void insertEventInSortedBuffer(const dvs_msgs::Event& e)
    {
        std::unique_lock<std::mutex> lock(events_mutex_);
        events_.push_back(e);
        // insertion sort to keep the buffer sorted
        // in practice, the events come almost always sorted,
        // so the number of iterations of this loop is almost always 0
        int j = (events_.size() - 1) - 1; // second to last element
        while(j >= 0 && events_[j].ts > e.ts)
        {
            events_[j+1] = events_[j];
            j--;
        }
        events_[j+1] = e;
    }

    cv::Size sensor_size_;

    // image flags
    bool got_first_image_;

    // pointers to most recent image and time
    ImageBuffer::iterator current_image_it_;
    ros::Time most_current_time_;
    
    // buffers for images and events
    EventBuffer events_;
    ImageBuffer images_;

    // ros
    ros::Subscriber event_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport it_;
    ros::NodeHandle nh_;

    // patch parameters
    Patches patches_;
    std::map<int, std::pair<cv::Mat, cv::Mat>> patch_gradients_;
    std::vector<int> lost_indices_;

    // delegation
    viewer::Viewer* viewer_ptr_ = NULL;
    nlls::Optimizer optimizer_;

    // mutex
    std::mutex events_mutex_;
    std::mutex images_mutex_;

    // tracks file
    std::ofstream tracks_file_;
};

}
