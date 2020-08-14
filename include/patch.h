#pragma once

#include <mutex>
#include <random>
#include <deque>

#include <opencv2/core.hpp>
#include <gflags/gflags.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <dvs_msgs/Event.h>

#include "error.h"
#include "types.h"

DECLARE_int32(patch_size);
DECLARE_int32(batch_size);
DECLARE_int32(update_every_n_events);


namespace tracker
{

/**
 * @brief The Patch struct which corresponds to an image patch defined by its center (feature position) and
 * dimension (patch size)
 */
struct Patch
{
    /**
   * @brief Patch constructor for patch. Is defined by its center, linear warping matrix, half size ( half the patch
   * side length and the direction of optical flow in its region.
   * @param center Position of the patch defined by the location of the associated corner
   * @param warping Affine 3x3 matrix describing the current transformation taking pixels in the patch to the initial image.
   * @param flow_angle Angle (rad) of the optical flow in the patch around the initial feature
   * @param half_size half size of the patch side length
   * @param t_init time stamp of the image where the corner was extracted
   */
    Patch(cv::Point2d center, ros::Time t_init):
        init_center_(center), flow_angle_(0), t_init_(t_init), t_curr_(t_init), event_counter_(0), color_(0, 0, 255),
        lost_(false), initialized_(false), tracking_quality_(1)
    {
        warping_ = cv::Mat::eye(3,3,CV_64F);

        half_size_ = (FLAGS_patch_size - 1) / 2;
        batch_size_ = FLAGS_batch_size;
        update_rate_ = FLAGS_update_every_n_events;

        reset(init_center_, t_init);
    }

    Patch() : Patch(cv::Point2f(-1,-1), ros::Time::now())
    {
        // contstructor for initializing lost features
        lost_ = true;
    }

    /**
   * @brief contains checks if event is contained in square around the current feature position
   */
    inline bool contains(double x, double y)
    {
        return half_size_ >= std::abs(x - center_.x) && half_size_ >= std::abs(y - center_.y);
    }

    /**
   * @brief insert an event in the event frame of the patch, incrementing the corresponding pixel by the polarity of the
   * event.
   */
    inline void insert(const dvs_msgs::Event& event)
    {
        event_buffer_.push_front(event);

        if (event_buffer_.size() > FLAGS_batch_size)
        {
            event_buffer_.pop_back();
        }

        event_counter_++;
    }

    /**
   * @brief warpPixel applies the inverse of the linear warp to unwarped and writes to warped.
   */
    inline void warpPixel(cv::Point2d unwarped, cv::Point2d &warped)
    {
        // compute the position of the feature according to the warp (equation (8) in the paper)
        cv::Mat W = warping_.inv();

        warped.x = W.at<double>(0,0) * unwarped.x + W.at<double>(0,1) * unwarped.y + W.at<double>(0,2);
        warped.y = W.at<double>(1,0) * unwarped.x + W.at<double>(1,1) * unwarped.y + W.at<double>(1,2);
    }

    /**
   * @brief getEventFramesAndReset returns the event_frame (according to equation (2) in the paper and resets the counter.
   */
    inline void getEventFramesAndReset(cv::Mat &event_frame)
    {
        // implements the function (2) in the paper
        event_frame = cv::Mat::zeros(2*half_size_+1, 2*half_size_+1, CV_64F);

        int iterations =  batch_size_< event_buffer_.size() ? batch_size_-1 : event_buffer_.size()-1;
        for (int i=0; i<iterations; i++)
        {
            dvs_msgs::Event &e = event_buffer_[i];

            if (!contains(e.x, e.y))
                continue;

            int x = e.x - center_.x + half_size_;
            int y = e.y - center_.y + half_size_;
            double rx = e.x - center_.x + half_size_ - x;
            double ry = e.y - center_.y + half_size_ - y;

            int increment = e.polarity ? 1 : -1;

            event_frame.at<double>(y+1, x+1) += increment * rx * ry;
            event_frame.at<double>(y  , x+1) += increment * rx * (1 - ry);
            event_frame.at<double>(y+1, x  ) += increment * (1 - rx) * ry;
            event_frame.at<double>(y  , x  ) += increment * (1 - rx) * (1 - ry);
        }

        // timstamp the image at the middle of the event batch
        t_curr_ = ros::Time(0.5*(event_buffer_[0].ts.toSec() + event_buffer_[iterations].ts.toSec()));
        event_counter_ = 0;
    }

    /**
     * @brief resets patch after it has been lost. 
     */
    inline void reset(cv::Point2d init_center, ros::Time t)
    {
        // reset feature after it has been lost
        event_counter_ = 0;
        tracking_quality_=1;
        lost_ = false;
        initialized_ = false;
        flow_angle_ = 0;
        
        center_ = init_center;
        init_center_ = init_center;
        t_curr_ = t;
        t_init_ = t;

        warping_ = cv::Mat::eye(3, 3, CV_64F);
        event_buffer_.clear();

        // set random color
        static std::uniform_real_distribution<double> unif(0, 255);
        static std::default_random_engine re;
        color_ = cv::Scalar(unif(re), unif(re), unif(re));

        static int id = 0;
        id_ = id++;
    }

    cv::Point2d init_center_;
    cv::Point2d center_;
    cv::Mat warping_;
    double flow_angle_;

    int half_size_;

    int event_counter_;

    ros::Time t_init_;
    ros::Time t_curr_;

    double tracking_quality_;
    cv::Scalar color_;

    int id_;
    int batch_size_;
    int update_rate_;
    bool lost_;
    bool initialized_;

    std::deque<dvs_msgs::Event> event_buffer_;
};

}
