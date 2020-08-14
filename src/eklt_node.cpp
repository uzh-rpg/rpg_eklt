#include <ros/ros.h>
#include <gflags/gflags.h>

#include "viewer.h"
#include "tracker.h"

// feature detection
DEFINE_int32(max_corners, 100, \
           "Maximum features allowed to be tracked.");
DEFINE_int32(min_corners, 60, \
           "Minimum features allowed to be tracked.");
DEFINE_int32(min_distance, 30, \
           "Minimum distance between detected features. Parameter passed to goodFeatureToTrack.");
DEFINE_int32(block_size, 30, \
           "Block size to compute Harris score. passed to harrisCorner and goodFeaturesToTrack.");

DEFINE_double(k, 0.04, \
              "Magic number for Harris score.");
DEFINE_double(quality_level, 0.3, \
              "Determines range of harris score allowed between the maximum and minimum. Passed to goodFeaturesToTrack.");
DEFINE_double(log_eps, 1e-2, \
              "Small constant to compute log image. To avoid numerical issues normally we compute log(img /255 + log_eps).");
DEFINE_double(first_image_t, -1, \
              "If specified discards all images until this time.");
DEFINE_string(tracks_file_txt, "", \
              "If specified, writes feature tracks to file with format id t x y.");

// tracker
DEFINE_int32(lk_window_size, 15, \
           "Parameter for KLT. Used for bootstrapping feature.");
DEFINE_int32(num_pyramidal_layers, 2, \
           "Parameter for KLT. Used for bootstrapping feature.");
DEFINE_int32(batch_size, 200, \
           "Determines the size of the event buffer for each patch. If a new event falls into a patch and the buffer is full, the older event in popped.");
DEFINE_int32(patch_size, 25, \
           "Determines size of patch around corner. All events that fall in this patch are placed into the features buffer.");
DEFINE_int32(max_num_iterations, 10, \
           "Maximum number of itrations allowed by the ceres solver to update optical flow direction and warp.");

DEFINE_double(displacement_px, 0.6, \
              "Controls scaling parameter for batch size calculation: from formula optimal batchsize == 1/Cth * sum |nabla I * v/|v||. displacement_px corresponds to factor 1/Cth");
DEFINE_double(tracking_quality, 0.4, \
              "minimum tracking quality allowed for a feature to be tracked. Can be a number between 0 (bad track) and 1 (good track). Is a rescaled number computed from ECC cost via tracking_quality == 1 - ecc_cost / 4. Note that ceres returns ecc_cost / 2.");
DEFINE_string(bootstrap, "",
              "Method for bootstrapping optical flow direction. Options are 'klt', 'events'");

// viewer
DEFINE_int32(update_every_n_events, 20, \
           "Updates viewer data every n events as they are tracked.");
DEFINE_double(scale, 4, \
           "Rescaling factor for image. Allows to see subpixel tracking.");
DEFINE_double(arrow_length, 5, \
           "Length of optical flow arrow.");

DEFINE_bool(display_features, true, \
           "Whether or not to display feature tracks.");
DEFINE_bool(display_feature_id, false, \
           "Whether or not to display feature ids.");
DEFINE_bool(display_feature_patches, false, \
           "Whether or not to display feature patches.");


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "eklt");
  ros::NodeHandle nh;

  viewer::Viewer viewer(nh);
  tracker::Tracker tracker(nh, viewer);

  ros::spin();


  return 0;
}
