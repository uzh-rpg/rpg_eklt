#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>

#include "viewer.h"

DECLARE_int32(max_corners);

DECLARE_double(scale);
DECLARE_double(arrow_length);

DECLARE_bool(display_features);
DECLARE_bool(display_feature_id);
DECLARE_bool(display_feature_patches);



namespace viewer
{

Viewer::Viewer(ros::NodeHandle& nh)
    : nh_(nh), got_first_image_(false), it_(nh)
{
}

Viewer::~Viewer()
{
    tracks_pub_.shutdown();
}

void Viewer::setViewData(tracker::Patches &patches, ros::Time &t, 
                         tracker::ImageBuffer::iterator image_it)
{
    // copy all patches data to feature_track_data_. This is later used to generate a preview of the feature tracks.
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i=0; i<patches.size(); i++)
    {
        if (feature_track_data_.patches.size() < patches.size())
        {
            feature_track_data_.patches.push_back(patches[i]);
        }
        else
        {
            feature_track_data_.patches[i] = patches[i];
        }
    }

    feature_track_data_.t = t;
    feature_track_data_.image = image_it->second;

    // signal that the first image has been received
    got_first_image_ = got_first_image_ || true;
}

void Viewer::initViewData(ros::Time t)
{
    // start viewer thread and publisher and allocate memory for feature_track_data_
    tracks_pub_ = it_.advertise("feature_tracks", 1);

    std::thread viewerThread(&Viewer::displayTracks, this);
    viewerThread.detach();

    int num_patches = FLAGS_max_corners;
    feature_track_data_.patches.reserve(num_patches);
    feature_track_data_.t = t;
    feature_track_data_.t_init = t;
}

void Viewer::publishImage(cv::Mat image, ros::Time stamp, std::string encoding, image_transport::Publisher pub)
{
    // helper for publishing images
    static cv_bridge::CvImage cv_image;

    cv_image.encoding = encoding;
    cv_image.image = image.clone();
    cv_image.header.stamp = stamp;

    pub.publish(cv_image.toImageMsg());
}

void Viewer::displayTracks()
{
    ros::Rate r(30);
    
    while (ros::ok())
    {
        //if the first image was not yet received do not do anything
        // otherwise prepare feature tracking preview
        r.sleep();
        if (!got_first_image_ || !FLAGS_display_features)
        {
            continue;
        }
        {
        // generate image with features and processed rates
        std::lock_guard<std::mutex> lock(data_mutex_);
        drawOnImage(feature_track_data_, feature_track_view_, feature_track_data_.image);
        publishImage(feature_track_view_, ros::Time::now(), "bgr8", tracks_pub_);
        }
    }
}

void Viewer::drawOnImage(FeatureTrackData& data, cv::Mat& view, cv::Mat& image)
{
    CHECK(image.size[0] > 0);
    const double& scale = FLAGS_scale;
    const double& arrow_length = FLAGS_arrow_length;
    const int& patch_size = FLAGS_patch_size;

    view.setTo(0);

    //convert to grayscale to 3channel U8
    cv::Mat c_image;
    cv::cvtColor(image, c_image, CV_GRAY2BGR);

    cv::Size size(scale*c_image.size[1], scale*c_image.size[0]);
    cv::resize(c_image, view, size, 0,0,cv::INTER_NEAREST);

    // draw timestamp
    std::string time_string = "t = " + std::to_string(data.t.toSec() - data.t_init.toSec());
    cv::putText(view, time_string, scale * (cv::Point(image.size[1], image.size[0]) - cv::Point(140,2)), cv::FONT_HERSHEY_COMPLEX_SMALL, scale*.7, cvScalar(0,255,255), 1, CV_AA);

    // draw patches
    for (int i=0; i<data.patches.size(); i++)
    {
        tracker::Patch& patch = data.patches[i];

        if (patch.lost_)
            continue;

        // draw patch center and lk feature with line between them
        cv::drawMarker(view, scale*patch.center_, patch.color_, cv::MARKER_CROSS, 10, 1);
        
        // draw arrow
        cv::Point2d flow_arrow_tip = patch.init_center_ + arrow_length * cv::Point2d(std::cos(patch.flow_angle_), std::sin(patch.flow_angle_));
        cv::Point2d warped_flow_arrow_tip;
        patch.warpPixel(flow_arrow_tip, warped_flow_arrow_tip);
        cv::arrowedLine(view, scale* patch.center_, scale * warped_flow_arrow_tip, cv::Scalar(255,255,0), 1, 8, 0, 0.1);

        int half_patch_size = (patch_size - 1) / 2;

        cv::Point2d top_left(patch.init_center_.x - half_patch_size, patch.init_center_.y - half_patch_size);
        cv::Point2d top_right(patch.init_center_.x + half_patch_size, patch.init_center_.y - half_patch_size);
        cv::Point2d bottom_left(patch.init_center_.x - half_patch_size, patch.init_center_.y + half_patch_size);
        cv::Point2d bottom_right(patch.init_center_.x + half_patch_size, patch.init_center_.y + half_patch_size);

        cv::Point2d top_middle = (top_left + top_right)/2;

        cv::Point2d top_left_warped, top_right_warped, bottom_left_warped, bottom_right_warped, top_middle_warped;
        patch.warpPixel(top_left, top_left_warped);
        patch.warpPixel(top_right, top_right_warped);
        patch.warpPixel(bottom_left, bottom_left_warped);
        patch.warpPixel(bottom_right, bottom_right_warped);
        patch.warpPixel(top_middle, top_middle_warped);

        // draw warped patch lines
        std::vector<cv::Point> corners = {scale*top_left_warped, scale*top_right_warped, scale*bottom_right_warped, scale*bottom_left_warped};

        if (patch.initialized_ && FLAGS_display_feature_patches)
        {
            cv::polylines(view, corners, true, patch.color_, 2);
            cv::line(view, scale*patch.center_, scale*top_middle_warped, patch.color_, 2);
        }

        // draw feature ids
        cv::Point text_pos = cv::Point(patch.center_.x-half_patch_size+2, patch.center_.y-half_patch_size+8);
        if (FLAGS_display_feature_id)
            cv::putText(view, std::to_string(i), scale*text_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, scale * 0.4, cvScalar(255,0,0), 2, CV_AA);
    }

}

}
