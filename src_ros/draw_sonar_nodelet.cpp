// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include <sstream>

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "acoustic_msgs/SonarImage.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// For uploading drawn sonar images to Image topic
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/SonarDrawer.h"
#include "sonar_image_proc/HistogramGenerator.h"

#include "sonar_image_proc/AbstractSonarInterface.h"

#include "sonar_image_msg_interface.h"

// Subscribes to sonar message topic, draws using opencv then publishes result

namespace draw_sonar {

using namespace std;
using namespace cv;

using std_msgs::UInt32MultiArray;

using sonar_image_proc::HistogramGenerator;

using sonar_image_proc::SonarColorMap;
using sonar_image_proc::InfernoColorMap;
using sonar_image_proc::InfernoSaturationColorMap;


class DrawSonarNodelet : public nodelet::Nodelet {
 public:
    DrawSonarNodelet()
      : Nodelet(),
        _maxRange(0.0),
        _colorMap( new InfernoColorMap ) // Set a reasonable default
    {;}

    virtual ~DrawSonarNodelet()
    {;}

 private:
    virtual void onInit() {
      ros::NodeHandle nh = getMTNodeHandle();
      ros::NodeHandle pnh = getMTPrivateNodeHandle();

      pnh.param<float>("max_range", _maxRange, 0.0);

      pnh.param<bool>("publish_old", _publishOldApi, false);
      pnh.param<bool>("publish_timing", _publishTiming, true);

      pnh.param<bool>("publish_histogram", _publishHistogram, false);

      std::string colorMapName;
      pnh.param<string>("color_map", colorMapName, "inferno");

      // Set color map
      setColorMap(colorMapName);

      if (_maxRange > 0.0) {
        NODELET_INFO_STREAM("Only drawing to max range " << _maxRange);
      }

      subSonarImage_ = nh.subscribe<acoustic_msgs::SonarImage>("sonar_image",
                            10, &DrawSonarNodelet::sonarImageCallback, this );

      pub_ = nh.advertise<sensor_msgs::Image>("drawn_sonar", 10);
      rectPub_ = nh.advertise<sensor_msgs::Image>("drawn_sonar_rect", 10);

      if (_publishOldApi)
        oldPub_ = nh.advertise<sensor_msgs::Image>("old_drawn_sonar", 10);

      if (_publishTiming)
        timingPub_ = nh.advertise<std_msgs::String>("sonar_image_proc_timing", 10);

      if (_publishHistogram)
        histogramPub_ = nh.advertise<std_msgs::UInt32MultiArray>("histogram", 10);

    }

    void cvBridgeAndPublish(const acoustic_msgs::SonarImage::ConstPtr &msg,
                              const cv::Mat &mat,
                              ros::Publisher &pub) {
        cv_bridge::CvImage img_bridge(msg->header,
                                      sensor_msgs::image_encodings::RGB8,
                                      mat);

        sensor_msgs::Image output_msg;
        img_bridge.toImageMsg(output_msg);
        pub.publish(output_msg);
    }

    void sonarImageCallback(const acoustic_msgs::SonarImage::ConstPtr &msg) {
      ROS_FATAL_COND(!_colorMap, "Colormap is undefined, this shouldn't happen");

      SonarImageMsgInterface interface(msg);

      ros::WallDuration oldApiElapsed, rectElapsed, mapElapsed, histogramElapsed;

      if (_publishOldApi) {
        ros::WallTime begin = ros::WallTime::now();

        // Used to be a configurable parameter, but now only meaningful
        // in the deprecated API
        const int pixPerRangeBin = 2;

        cv::Size sz = sonar_image_proc::old_api::calculateImageSize(interface,
                                                  cv::Size(0, 0), pixPerRangeBin,
                                                  _maxRange);
        cv::Mat mat(sz, CV_8UC3);
        mat = sonar_image_proc::old_api::drawSonar(interface, mat, *_colorMap, _maxRange);

        cvBridgeAndPublish(msg, mat, oldPub_);

        oldApiElapsed = ros::WallTime::now() - begin;
      }

      if (_publishHistogram) {
        ros::WallTime begin = ros::WallTime::now();

        auto histogramOut = UInt32MultiArray();
        histogramOut.data = HistogramGenerator::Generate(interface);

        histogramPub_.publish(histogramOut);

        histogramElapsed = ros::WallTime::now() - begin;
      }

      {
        ros::WallTime begin = ros::WallTime::now();

        cv::Mat rectMat = _sonarDrawer.drawRectSonarImage(interface, *_colorMap);

        // Rotate rectangular image to the more expected format where zero range
        // is at the bottom of the image, with negative azimuth to the right
        // aka (rotated 90 degrees CCW)
        cv::Mat rotatedRect;
        cv::rotate(rectMat, rotatedRect, cv::ROTATE_90_COUNTERCLOCKWISE);
        cvBridgeAndPublish(msg, rotatedRect, rectPub_);

        rectElapsed = ros::WallTime::now() - begin;
        begin = ros::WallTime::now();

        cv::Mat sonarMat = _sonarDrawer.remapRectSonarImage(interface, rectMat);
        cvBridgeAndPublish(msg, sonarMat, pub_);

        mapElapsed = ros::WallTime::now() - begin;
      }



      if (_publishTiming) {
        ostringstream output;

        output << "{";
        output << "\"draw_total\" : " << (mapElapsed + rectElapsed).toSec();
        output << ", \"rect\" : " << rectElapsed.toSec();
        output << ", \"map\" : " << mapElapsed.toSec();

        if (_publishOldApi)
          output << ", \"old_api\" : " << oldApiElapsed.toSec();
        if (_publishHistogram)
          output << ", \"histogram\" : " << histogramElapsed.toSec();

        output << "}";

        std_msgs::String out_msg;
        out_msg.data = output.str();

        timingPub_.publish(out_msg);
      }
    }

    void setColorMap(const std::string &colorMapName) {
      // TBD actually implement the parameter processing here...
      _colorMap.reset( new InfernoSaturationColorMap() );
    }


    ros::Subscriber subSonarImage_;
    ros::Publisher pub_, rectPub_, oldPub_, timingPub_, histogramPub_;

    sonar_image_proc::SonarDrawer _sonarDrawer;

    float _maxRange;
    bool _publishOldApi, _publishTiming, _publishHistogram;

    std::unique_ptr< SonarColorMap > _colorMap;
};

}  // namespace draw_sonar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draw_sonar::DrawSonarNodelet, nodelet::Nodelet);
