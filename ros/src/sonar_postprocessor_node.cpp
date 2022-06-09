// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "nodelet/loader.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sonar_postprocessor");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load(ros::this_node::getName(),
               "sonar_image_proc/sonar_postprocessor", remap, nargv);

  ros::spin();
  return 0;
}
