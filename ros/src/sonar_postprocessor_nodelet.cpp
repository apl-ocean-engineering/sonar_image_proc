// Copyright 2021-2022 University of Washington Applied Physics Laboratory
// Author: Aaron Marburg

#include <sstream>

#include "acoustic_msgs/ProjectedSonarImage.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sonar_image_proc/sonar_image_msg_interface.h"

namespace sonar_postprocessor {

using acoustic_msgs::ProjectedSonarImage;
using sonar_image_proc::SonarImageMsgInterface;

class SonarPostprocessorNodelet : public nodelet::Nodelet {
public:
  SonarPostprocessorNodelet() : Nodelet() { ; }

  virtual ~SonarPostprocessorNodelet() { ; }

private:
  virtual void onInit() {
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle pnh = getMTPrivateNodeHandle();

    pnh.param<float>("gain", gain_, 1.0);
    pnh.param<float>("gamma", gamma_, 0.0);

    subSonarImage_ = nh.subscribe<ProjectedSonarImage>(
        "sonar_image", 10, &SonarPostprocessorNodelet::sonarImageCallback,
        this);

    pubSonarImage_ = nh.advertise<ProjectedSonarImage>("sonar_image_postproc", 10);

    ROS_DEBUG("sonar_processor ready to run...");
  }

  void sonarImageCallback(const acoustic_msgs::ProjectedSonarImage::ConstPtr &msg) {
    SonarImageMsgInterface interface(msg);

    // For now, only postprocess 32bit images
    if (msg->image.dtype != msg->image.DTYPE_UINT32) {
      pubSonarImage_.publish(msg);
      return;
    }

    // Expect this will copy
    acoustic_msgs::ProjectedSonarImage out = *msg;

    // For now, only 8-bit output is supported
    out.image.dtype = out.image.DTYPE_UINT8;
    out.image.data.clear();
    out.image.data.reserve(interface.ranges().size() *
                           interface.azimuths().size());

    for (unsigned int r_idx = 0; r_idx < interface.nRanges(); ++r_idx) {
      for (unsigned int a_idx = 0; a_idx < interface.nAzimuth(); ++a_idx) {
        sonar_image_proc::AzimuthRangeIndices idx(a_idx, r_idx);

        // Avoid log(0)
        auto intensity = interface.intensity_uint32(idx);
        auto vv = log(std::max((uint)1, intensity)) / log(UINT32_MAX);

        // The output image will look better if the full range of the colormap
        // corresponds to a subset of the range of v.
        const float vmax = 1.0, threshold = 0.74;
        auto color = (vv - threshold) / (vmax - threshold);
        color = std::min(1.0, std::max(0.0, vv));
        out.image.data.push_back(UINT8_MAX * color);
      }
    }
    pubSonarImage_.publish(out);
  }

protected:
  ros::Subscriber subSonarImage_;
  ros::Publisher pubSonarImage_;

  float gain_, gamma_;
};

} // namespace sonar_postprocessor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sonar_postprocessor::SonarPostprocessorNodelet,
                       nodelet::Nodelet);
