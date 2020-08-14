#pragma once

#include <deque>

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <dvs_msgs/Event.h>


namespace tracker
{

struct Patch; //forward decl
using Patches = std::vector<Patch>; //forward decl

using EventBuffer = std::deque<dvs_msgs::Event>;
using ImageBuffer = std::map<ros::Time, cv::Mat>;

}

namespace viewer
{

struct FeatureTrackData
{
    tracker::Patches patches;
    ros::Time t, t_init;
    cv::Mat image;
};

}

namespace nlls
{

using Grid = ceres::Grid2D<double, 2>;
using GridPtr = std::unique_ptr<Grid>;
using Interpolator = ceres::BiCubicInterpolator<Grid>;
using InterpolatorPtr = std::unique_ptr<ceres::BiCubicInterpolator<Grid>>;

struct OptimizerDatum
{
    OptimizerDatum() {}
    OptimizerDatum(std::vector<double> &grad, cv::Mat& img, int num_patches)
    {
        grad_ = grad;
        grad_grid_ = new nlls::Grid(grad_.data(), 0, img.rows, 0, img.cols);
        grad_interp_ = new nlls::Interpolator(*grad_grid_);
        ref_counter_ = num_patches;
    }

    void clear()
    {
         delete grad_interp_;
         delete grad_grid_;
    }

    std::vector<double> grad_;
    nlls::Grid* grad_grid_;
    nlls::Interpolator* grad_interp_;
    int ref_counter_;
};

using OptimizerData = std::map<ros::Time, OptimizerDatum>;

}



