
#include <gflags/gflags.h>

#include "optimizer.h"

DECLARE_int32(patch_size);
DECLARE_int32(max_num_iterations);
DECLARE_double(log_eps);


namespace nlls
{

Optimizer::Optimizer() :
         patch_size_(FLAGS_patch_size)
{
    prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

    solver_options.minimizer_progress_to_stdout = false;
    solver_options.num_threads = 1;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.logging_type = ceres::SILENT;
    solver_options.max_num_iterations = FLAGS_max_num_iterations;
    solver_options.use_nonmonotonic_steps = true;

    loss_function = NULL;
}

Optimizer::~Optimizer()
{
    for (auto &it : optimizer_data_) 
        it.second.clear();
    delete cost_function_;
}

void Optimizer::decrementCounter(ros::Time& time)
{
    // keep track of reference counter to release gradient data
    if (optimizer_data_.find(time) == optimizer_data_.end())
        return;

    --optimizer_data_[time].ref_counter_;
    if (optimizer_data_[time].ref_counter_ == 0)
    {
         optimizer_data_[time].clear();
         optimizer_data_.erase(time);
    }
}

void Optimizer::getLogGradients(const cv::Mat& img, cv::Mat& I_x, cv::Mat& I_y)
{
    // compute log gradients of image
    const double& log_eps = FLAGS_log_eps;

    cv::Mat normalized_image, log_image;
    img.convertTo(normalized_image, CV_64F, 1.0 / 255.0);
    cv::log(normalized_image + log_eps, log_image);

    cv::Sobel( log_image / 8, I_x, CV_64F, 1, 0, 3);
    cv::Sobel( log_image / 8, I_y, CV_64F, 0, 1, 3);
}

bool Optimizer::precomputeLogImageArray(const tracker::Patches& patches, const tracker::ImageBuffer::iterator& image_it)
{
    cv::Mat I_x, I_y;
    getLogGradients(image_it->second, I_x, I_y);

    std::vector<double> grad;

    // initialize grid that is used by ceres interpolator
    for (int row=0; row<image_it->second.rows;row++)
    {
        for (int col=0; col<image_it->second.cols;col++)
        {
            grad.push_back(I_x.at<double>(row,col));
            grad.push_back(I_y.at<double>(row,col));
        }
    }

    // store this for later use
    const ros::Time& t = image_it->first;
    optimizer_data_[t] = OptimizerDatum(grad, image_it->second, patches.size());
    return true;
}

void Optimizer::optimizeParameters(const cv::Mat &event_frame, tracker::Patch &patch)
{
    double norm=0;

    ceres::Problem problem(prob_options);
    ceres::Solver::Summary summary;

    // for now 3 free parameters for warp, x, y translation and theta rotation
    double p0[3];
    ErrorRotation::getP0(p0, patch.warping_);
    double v0[] = {patch.flow_angle_};

    // create cost functor that depends on the event frame, and current and initial
    // patch location
    ErrorRotation* functor;
    cost_function_ = Generator::Create(patch.center_,
                                       patch.init_center_, 
                                       &event_frame, 
                                       optimizer_data_[patch.t_init_].grad_interp_, 
                                       functor);

    problem.AddResidualBlock(cost_function_, NULL, p0, v0);

    ceres::Solve(solver_options, &problem, &summary);

    // update patch according to new optimizer
    cv::Mat camera_warp;
    ErrorRotation::getWarp(p0, camera_warp);

    delete functor;

    // update state of patch
    patch.warping_ = camera_warp.clone();
    patch.flow_angle_ = fmod(v0[0] , 2 * M_PI);

    //remap tracking cost to 0-1 (1 is best and 0 is bad)
    patch.tracking_quality_ = 1 - summary.final_cost / 2;
    patch.warpPixel(patch.init_center_, patch.center_);
}

};
