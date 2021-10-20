#pragma once

#include <mutex>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <ros/ros.h>

#include "patch.h"
#include "types.h"


namespace viewer
{
/**
 * @brief viewer object that displays the feature tracks as they are computed.
 */
class Viewer
{
public:
    Viewer(ros::NodeHandle& nh);
    ~Viewer();
    /**
     * @brief main running thread
     */
    void displayTracks();
    /**
     * @brief initializes the tracking data that is used to generate a preview
     */
    void initViewData(ros::Time t);    
    /**
     * @brief Used to set the tracking data
     */
    void setViewData(tracker::Patches &patches, ros::Time &t, 
                     tracker::ImageBuffer::iterator image_it);
private:
    /**
     * @brief function that draws on image
     */
    void drawOnImage(FeatureTrackData& data, cv::Mat& view, cv::Mat& image);
    /**
     * @brief helper function to publish image
     */
    void publishImage(cv::Mat image, ros::Time stamp, std::string encoding, image_transport::Publisher pub);

    ros::NodeHandle nh_;

    FeatureTrackData feature_track_data_;

    cv::Mat feature_track_view_;

    image_transport::Publisher tracks_pub_;
    image_transport::ImageTransport it_;

    std::mutex data_mutex_;

    bool got_first_image_;
};

}
