// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include <sstream>

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "acoustic_msgs/SonarImage.h"
#include "sonar_image_proc/sonar_image_msg_interface.h"



namespace sonar_postprocessor {

using sonar_image_proc::SonarImageMsgInterface;
using acoustic_msgs::SonarImage;

class SonarPostprocessorNodelet : public nodelet::Nodelet {
 public:
    SonarPostprocessorNodelet()
      : Nodelet()
    {;}

    virtual ~SonarPostprocessorNodelet()
    {;}

 private:
    virtual void onInit() {
      ros::NodeHandle nh = getMTNodeHandle();
      ros::NodeHandle pnh = getMTPrivateNodeHandle();

      pnh.param<float>("gain", gain_, 0.0);

      subSonarImage_ = nh.subscribe<SonarImage>("sonar_image",
                            10, &SonarPostprocessorNodelet::sonarImageCallback, this);

      pubSonarImage_ = nh.advertise<SonarImage>("sonar_image_postproc", 10);
    }

    void sonarImageCallback(const acoustic_msgs::SonarImage::ConstPtr &msg) {
      SonarImageMsgInterface interface(msg);

      pubSonarImage_.publish(msg);
    }


    ros::Subscriber subSonarImage_;
    ros::Publisher pubSonarImage_;

    float gain_;
};

}  // namespace draw_sonar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sonar_postprocessor::SonarPostprocessorNodelet, nodelet::Nodelet);
