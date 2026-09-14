
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <chrono>

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
// #include <cv_bridge/cv_bridge.h>
#include "sonar_image_proc/ColorMaps.h"
// src/sonar_image_proc/include/sonar_image_proc/ColorMaps.h
#include "sonar_image_proc/DrawSonar.h"
#include "sonar_image_proc/HistogramGenerator.h"
#include "sonar_image_proc/SonarDrawer.h"
#include "sonar_image_proc/sonar_image_msg_interface.h"

// Subscribes to sonar message topic, draws using opencv then publishes
// result

namespace draw_sonar {

  class DrawSonarComponent : public rclcpp::Node
{
  public:
  explicit DrawSonarComponent(const rclcpp::NodeOptions &options)
      : Node("draw_sonar_component", options)

    {
          max_range_          = this->declare_parameter<double>("max_range", 0.0);
          publish_old_api_    = this->declare_parameter<bool>("publish_old", false);
          publish_timing_     = this->declare_parameter<bool>("publish_timing", true);
          publish_histogram_  = this->declare_parameter<bool>("publish_histogram", false);
          color_map_name_     = this->declare_parameter<std::string>("color_map", "inferno");
          // dynamic parameter change 
          // "Minimum intensity in log scale (in db, 0 for full range)", 0, -221, 0
          min_db_ = this->declare_parameter<double>("min_db", 0.0);
          // "Maximum intensity in log scale (in db, 0 for full range)", 0, -221, 0
          max_db_ = this->declare_parameter<double>("max_db", 0.0);
          log_scale_ = this->declare_parameter<bool>("log_scale", false); //Use log scale for intensity
          range_spacing_ = this->declare_parameter<double>("range_spacing", 0.0); //"Spacing of range marks (0 for auto)", 0,  0, 100
          bearing_spacing_ = this->declare_parameter<double>("bearing_spacing", 45.0); // "Spacing of bearing radials", 20,  0, 100
          line_alpha_ = this->declare_parameter<double>("line_alpha", 0.2); //"Alpha for lines", 0.5,  0, 1.0
          line_thickness_ = this->declare_parameter<int>("line_thickness", 1); //"Line thickness", 1, 1, 5

          setColorMap(color_map_name_); 

          if (max_range_>0.0) {
            RCLCPP_INFO(get_logger(), "Only drawing to max range %.2f", max_range_);
          }
        
          sub_sonar_image_ = this->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>("sonar_image", 10, std::bind(&DrawSonarComponent::sonarImageCallback, this, std::placeholders::_1));

          // Publishers
          pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar", 10);
          rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_rect", 10);
          osd_pub_     = this->create_publisher<sensor_msgs::msg::Image>("drawn_sonar_osd", 10);

          if (publish_old_api_) {
            old_pub_   = this->create_publisher<sensor_msgs::msg::Image>("old_drawn_sonar", 10);
          }


          if (publish_timing_) {
            timing_pub_= this->create_publisher<std_msgs::msg::String>("sonar_image_proc_timing", 10);
          }

          if (publish_histogram_) {
            histogram_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("histogram", 10);
          }

          param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DrawSonarComponent::on_param_change, this, std::placeholders::_1)
          );

          // Add parameter change callback for "dynamic reconfigure"
          // this->add_on_set_parameters_callback(
          //   [this](const std::vector<rclcpp::Parameter> &params) {
          //     rcl_interfaces::msg::SetParametersResult result;
          //     result.successful = true;
          //     this->update_sonar_config();
          //     return result;
          //   });

          RCLCPP_INFO(this->get_logger(), "draw_sonar_component node started.");
    }
    private:
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters) {
            if (param.get_name() == "min_db") min_db_ = param.as_double();
            else if (param.get_name() == "max_db") max_db_ = param.as_double();
            else if (param.get_name() == "log_scale") log_scale_ = param.as_bool();
            else if (param.get_name() == "range_spacing")
                range_spacing_ = param.as_double();
            else if (param.get_name() == "bearing_spacing")
                bearing_spacing_ = param.as_double();
            else if (param.get_name() == "line_alpha")
                line_alpha_ = param.as_double();
            else if (param.get_name() == "line_thickness")
                line_thickness_ = param.as_double();

            // Apply to sonar_drawer or other internal classes as needed:
            sonar_drawer_.overlayConfig().setRangeSpacing(range_spacing_);
            sonar_drawer_.overlayConfig().setRadialSpacing(bearing_spacing_);
            sonar_drawer_.overlayConfig().setLineAlpha(line_alpha_);
            sonar_drawer_.overlayConfig().setLineThickness(line_thickness_);
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void setColorMap(const std::string &colorMapName) {
      // TBD actually implement the parameter processing here...
      (void)colorMapName;
      _colorMap.reset(new sonar_image_proc::InfernoSaturationColorMap());
    }

    void cvBridgeAndPublish(
        const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
        const cv::Mat &mat,
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub)
    {
      auto msg_out = cv_bridge::CvImage(msg->header, "rgb8", mat).toImageMsg();
      pub->publish(*msg_out); // or: pub->publish(msg_out);
    }


    void sonarImageCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg){

      if(!_colorMap){
        RCLCPP_FATAL(this->get_logger(), "Colormap is undefined, this shouldn't happen");
        return;
      }

      sonar_image_proc::SonarImageMsgInterface interface(msg);
      
      if (log_scale_) {
      interface.do_log_scale(min_db_, max_db_);
      }

      double oldApiElapsed = 0, rectElapsed = 0, mapElapsed = 0, histogramElapsed=0;

      if (publish_old_api_) {
        auto begin = std::chrono::steady_clock::now();

        // Used to be a configurable parameter, but now only meaningful
        // in the deprecated API
        const int pixPerRangeBin = 2;

        cv::Size sz = sonar_image_proc::old_api::calculateImageSize(
            interface, cv::Size(0, 0), pixPerRangeBin, max_range_);
        cv::Mat mat(sz, CV_8UC3);
        mat = sonar_image_proc::old_api::drawSonar(interface, mat, *_colorMap,
                                                  max_range_);

        // cv::Mat mat = sonar_image_proc::old_api::drawSonar(interface, cv::Mat(sz, CV_8UC3), *_colorMap, max_range_);

        cvBridgeAndPublish(msg, mat, old_pub_);

        oldApiElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
      }

      if (publish_histogram_){
        auto begin = std::chrono::steady_clock::now();
        std_msgs::msg::UInt32MultiArray histogramOut;
        histogramOut.data = histogram_generator_.Generate(interface);

        histogram_pub_->publish(histogramOut);
        histogramElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
      }

      {
        auto begin = std::chrono::steady_clock::now();

        cv::Mat rectMat = sonar_drawer_.drawRectSonarImage(interface, *_colorMap);

        cv::Mat rotatedRect;
        cv::rotate(rectMat, rotatedRect, cv::ROTATE_90_COUNTERCLOCKWISE);
        cvBridgeAndPublish(msg, rotatedRect, rect_pub_);

        rectElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

        begin = std::chrono::steady_clock::now();

        cv::Mat sonarMat = sonar_drawer_.remapRectSonarImage(interface, rectMat);
        cvBridgeAndPublish(msg, sonarMat, pub_);

        if (osd_pub_->get_subscription_count() > 0) {  // Note: ROS2 API
          cv::Mat osdMat = sonar_drawer_.drawOverlay(interface, sonarMat);
          cvBridgeAndPublish(msg, osdMat, osd_pub_);
        }

        mapElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
      }


      if (publish_timing_) {
        std::ostringstream output;

        output << "{";
        output << "\"draw_total\" : " << (mapElapsed + rectElapsed);
        output << ", \"rect\" : " << rectElapsed;
        output << ", \"map\" : " << mapElapsed;

        if (publish_old_api_) output << ", \"old_api\" : " << oldApiElapsed;
        if (publish_histogram_) output << ", \"histogram\" : " << histogramElapsed;

        output << "}";

        std_msgs::msg::String out_msg;
        out_msg.data = output.str();

        timing_pub_->publish(out_msg);
      }

    }

    sonar_image_proc::SonarDrawer sonar_drawer_;
    sonar_image_proc::HistogramGenerator histogram_generator_;
    // sonar_image_proc::SonarColorMap sonar_color_map_;
    std::unique_ptr<sonar_image_proc::SonarColorMap> _colorMap;
    float max_range_;
    float min_db_, max_db_;
    float range_spacing_, bearing_spacing_;
    float line_alpha_, line_thickness_;
    bool log_scale_;
    bool publish_old_api_, publish_timing_, publish_histogram_;
    std::string color_map_name_;

    rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sub_sonar_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_, rect_pub_, osd_pub_, old_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr histogram_pub_;

    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_callback_handle_;

};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(draw_sonar::DrawSonarComponent)

