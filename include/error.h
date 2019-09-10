#pragma once

#include <opencv2/core/core.hpp>
#include <ceres/cubic_interpolation.h>
#include <ceres/ceres.h>

#include "types.h"


namespace nlls
{
/**
 * @brief Config which contains all the arguments that are used to compute the loss.
 */
struct CostFunctionConfig
{
    CostFunctionConfig(cv::Point2d feature, 
                       cv::Point2d init_feature, 
                       const cv::Mat* event_frame, 
                       Interpolator* grad_interp)
        :feature_(feature), event_frame_(event_frame), grad_interp_(grad_interp), init_feature_(init_feature)
    {
        patch_size_ = event_frame_->size[0];
        half_size_ = (event_frame_->size[0]-1)/2;
        size_ = patch_size_*patch_size_;
        norm_event_frame_ = cv::norm(*event_frame_);
    }

    cv::Point2d feature_;
    cv::Point2d init_feature_;
    const cv::Mat* event_frame_;

    int patch_size_;
    int half_size_;
    int size_;
    int height_;
    int width_;

    Interpolator* grad_interp_;

    double norm_event_frame_;
};

/**
 * @brief Implements the cost function in equation (7) of the paper.
 */
struct ErrorRotation
{
    
    ErrorRotation(CostFunctionConfig* config) : config_(config) {}

    static void getP0(double p[], const cv::Mat& warp)
    {
        p[0] = std::atan2(warp.at<double>(1,0), warp.at<double>(0,0));
        p[1] = warp.at<double>(0,2);
        p[2] = warp.at<double>(1,2);
    }

    static void getWarp(double p0[], cv::Mat& warp)
    {
        warp = (cv::Mat_<double>(3,3) << std::cos(p0[0]), -std::sin(p0[0]), p0[1],
                std::sin(p0[0]),  std::cos(p0[0]), p0[2],
                0,                0,               1);
    }

    template<typename T>
    bool operator()(const T* p, const T* v, T* residual) const
    {
        // warp image gradient
        T norm_sq(1e-3);
        warp(p, v, residual, norm_sq);

        // compute loss function in equation (7) of the paper
        // we need to do 2 passes since we need to normalize by the gradient
        for (int i=0;i<config_->size_;i++)
        {
            residual[i] = residual[i] / ceres::sqrt(norm_sq) +
                    config_->event_frame_->at<double>(i / config_->patch_size_, i % config_->patch_size_) / config_->norm_event_frame_;
        }
        return true;
    }

    template<typename T>
    void warp(const T* p, const T* v, T* residual, T &norm_sq) const
    {
        // remap the image gradient according to equation (4)
        const T& r = p[0], tx = p[1], ty = p[2];

        T vx = ceres::cos(v[0]);
        T vy = ceres::sin(v[0]);

        T a0 = ceres::cos(p[0]);
        T a1 = -ceres::sin(p[0]);

        T a2 = -a1;
        T a3 = a0;

        for (int y=0;y<config_->patch_size_;y++)
        {
            for (int x=0;x<config_->patch_size_;x++)
            {
                T x_p = a0*T(x) + a1*T(y) + a0*(config_->feature_.x - config_->half_size_) + a1 * (config_->feature_.y - config_->half_size_) + p[1];
                T y_p = a2*T(x) + a3*T(y) + a2*(config_->feature_.x - config_->half_size_) + a3 * (config_->feature_.y - config_->half_size_) + p[2];

                T g[2];
                config_->grad_interp_->Evaluate(y_p, x_p, g);
                residual[x + config_->patch_size_ * y] = g[0] * vx + g[1] * vy;
                norm_sq += ceres::pow(residual[x + config_->patch_size_ * y], 2);
            }
        }
    }

    CostFunctionConfig* config_;
};


struct Generator
{
  static ceres::CostFunction* Create(cv::Point2d feature, 
                                     cv::Point2d init_feature, 
                                     const cv::Mat* event_frame, 
                                     Interpolator* grad_interp, 
                                     ErrorRotation* &functor)
  {
      CostFunctionConfig* config = new CostFunctionConfig(feature, init_feature, event_frame, grad_interp);
      functor = new ErrorRotation(config);
      int size = pow(event_frame->size[0], 2);
      return new ceres::AutoDiffCostFunction<ErrorRotation, ceres::DYNAMIC, 3, 1>(functor, size);
  }
};

} // nlls
