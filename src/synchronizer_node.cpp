#include <ros/ros.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "synchronizer.h"
#include "synchronizer_online.h"

DEFINE_string(input_video_path, "", "Absolute path to video that should be synchronized.");
DEFINE_string(input_bag_path, "/home/dani/projects/dual_setup_calibration/data/calibration_data/test.bag", "Absolute path to bag that should be synchronized.");
DEFINE_string(image_topic, "/flir/image_raw", "Image topic in final bag.");
DEFINE_string(trigger_topic, "/prophesee/camera/extTrigger", "Method for bootstrapping optical flow direction. Options are 'klt', 'events'");
DEFINE_string(input_image_topic, "/flir/image_raw", "Method for bootstrapping optical flow direction. Options are 'klt', 'events'");
DEFINE_bool(online, true, "Method for bootstrapping optical flow direction. Options are 'klt', 'events'");
DEFINE_string(event_topic, "/prophesee/camera/cd_events_buffer", "Method for bootstrapping optical flow direction. Options are 'klt', 'events'");


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "synchronizer");
  ros::NodeHandle nh;
   

  if (!FLAGS_online)
  {
    synchronizer::Synchronizer synchronizer(FLAGS_input_video_path, 
                                            FLAGS_input_bag_path, 
                                            FLAGS_image_topic, 
                                            FLAGS_trigger_topic,
                                            FLAGS_input_image_topic);
  }
  else
  {
    synchronizer::SynchronizerOnline synchronizer_online(nh, FLAGS_image_topic, 
                                                         FLAGS_trigger_topic,
                                                         FLAGS_event_topic);

    ros::spin();
    
  }
  



  return 0;
}
