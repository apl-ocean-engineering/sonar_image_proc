# libdraw_sonar

Code to draw data from forward-looking imaging sonars. 

If built with `catkin` for ROS, it will enable support for [acoustic_msgs](https://github.com/apl-ocean-engineering/hydrographic_msgs/tree/main/acoustic_msgs) and build a ROS node/nodelet [draw_sonar](https://github.com/apl-ocean-engineering/libdraw_sonar/tree/master/src_ros) which takes a [SonarImage](https://github.com/apl-ocean-engineering/hydrographic_msgs/blob/main/acoustic_msgs/msg/SonarImage.msg) and produces and [Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html).

# License

Licensed under [BSD 3-clause license](LICENSE).
