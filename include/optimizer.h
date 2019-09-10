#pragma once

#include <ros/ros.h>

#include "error.h"
#include "patch.h"
#include "types.h"


namespace nlls
{

struct Optimizer
/**
 * @brief The Optimizer performs optimization to find the best warp and optical flow for each patch.
 */
{
    Optimizer();
    ~Optimizer();

    /**
     * @brief Counts how many features are using the current image with timestamp time. 
     * If none are using it, free the memory.
     */
    void decrementCounter(ros::Time& time);

    /**
     * @ brief precomputes log gradients of the image
     */
    void getLogGradients(const cv::Mat& img, cv::Mat& I_x, cv::Mat& I_y);
    bool precomputeLogImageArray(const tracker::Patches& patches, const tracker::ImageBuffer::iterator& image_it);
    
    /**
     * @brief perform optimization of cost function (7) in the original paper.
     */
    void optimizeParameters(const cv::Mat &event_frame, tracker::Patch &patch);

    ceres::Problem::Options prob_options;
    ceres::Solver::Options solver_options;
    ceres::LossFunction* loss_function;

    OptimizerData optimizer_data_;

    int patch_size_;
    ceres::CostFunction* cost_function_;
};

}
