# sonar_image_proc

Library to draw data from forward-looking imaging sonars.

This repo is designed to build in Colcon (ROS2), Catkin (ROS1) and cmake environments, but it contains **no** ROS-specific code.  ROS1,2 wrappers are found in [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc)

# API

Sonar drawing is implemented in the [SonarDrawer](include/sonar_image_proc/SonarDrawer.h) class, which takes an instance of an [AbstractSonarInterface](include/sonar_image_proc/AbstractSonarInterface.h) and returns a cv::Mat.   SonarDrawer computes and stores pre-calculated matrices to accelerate the drawing.

A convenience function [drawSonar](include/sonar_image_proc/DrawSonar.h) is also provided.  It is a trivial wrapper which creates an instance of SonarDrawer then calls it.  Calls to drawSonar do not retain the cached matrices and are less efficient.

# Related Packages

* [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc) provides ROS{1,2} nodes for creating images from sonar data.
* [liboculus](https://github.com/apl-ocean-engineering/liboculus) provides network IO and data parsing for the Oculus sonar (non-ROS).
* [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver) provides a ROS node for interfacing with the Oculus sonar.
* [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs) defines the ROS [ProjectedSonarImage](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs/msg/ProjectedSonarImage.msg) message type published by [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver).
* [rqt_sonar_image_view](https://github.com/apl-ocean-engineering/rqt_sonar_image_view) is an Rqt plugin for displaying sonar imagery (uses [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc))


# License

Licensed under [BSD 3-clause license](LICENSE).
