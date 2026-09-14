#include "sonar_image_proc/draw_sonar_lifecycle_node.hpp"

namespace draw_sonar {
    DrawSonarLifecycleNode::DrawSonarLifecycleNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("draw_sonar_node", options)
      {
        RCLCPP_INFO(get_logger(), "Creating sonar draw node");

        // Initialize parameters from YAML setup
        // this->declare_parameter<double>("max_range", 50.0);
        max_range_ = declare_parameter<double>("max_range", 30.0);
        publish_old_api_ = declare_parameter<bool>("publish_old", false);
        publish_timing_ = declare_parameter<bool>("publish_timing", true);
        publish_histogram_ = declare_parameter<bool>("publish_histogram", false);
        color_map_name_ = declare_parameter<std::string>("color_map", "inferno");
        
        line_alpha_ = declare_parameter<double>("line_alpha", 0.5);
        line_thickness_ = declare_parameter<int>("line_thickness", 1);
        range_spacing_ = declare_parameter<double>("range_spacing", 0.0);
        bearing_spacing_ = declare_parameter<double>("bearing_spacing", 20.0);
        log_scale_ = declare_parameter<bool>("log_scale", false);
        min_db_ = declare_parameter<double>("min_db", 0.0);
        max_db_ = declare_parameter<double>("max_db", 0.0);
        RCLCPP_DEBUG(get_logger(), "Constructor done - parameters declared");
      }

    void DrawSonarLifecycleNode::applyOverlayConfig()
    {

        sonar_drawer_.overlayConfig()
            .setRangeSpacing(range_spacing_)
            .setRadialSpacing(bearing_spacing_)
            .setLineAlpha(line_alpha_)
            .setLineThickness(line_thickness_);

    }

    rcl_interfaces::msg::SetParametersResult
    DrawSonarLifecycleNode::on_param_change(const std::vector<rclcpp::Parameter> &parameters)
    {

        for (const auto &param : parameters) {
            const auto &name = param.get_name();
            if (name == "min_db") min_db_ = param.as_double();
            else if (name == "max_db") max_db_ = param.as_double();
            else if (name == "log_scale") log_scale_ = param.as_bool();
            else if (name == "range_spacing")
              range_spacing_ = param.as_double();
            else if (name == "bearing_spacing")
              bearing_spacing_ = param.as_double();
            else if (name == "line_alpha")
              line_alpha_ = param.as_double();
            else if (name == "line_thickness")
              line_thickness_ = param.as_int();

            // Apply to sonar_drawer or other internal classes as needed:
            // sonar_drawer_.overlayConfig().setRangeSpacing(range_spacing_);
            // sonar_drawer_.overlayConfig().setRadialSpacing(bearing_spacing_);
            // sonar_drawer_.overlayConfig().setLineAlpha(line_alpha_);
            // sonar_drawer_.overlayConfig().setLineThickness(line_thickness_);

            applyOverlayConfig(); 
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void DrawSonarLifecycleNode::setColorMap(const std::string &colorMapName) {
      // TBD actually implement the parameter processing here...
      (void)colorMapName;
      _colorMap.reset(new sonar_image_proc::InfernoSaturationColorMap());
    }

    void DrawSonarLifecycleNode::cvBridgeAndPublish(
    const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
    const cv::Mat &mat,
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr &pub)
        {
        auto msg_out = cv_bridge::CvImage(msg->header, "rgb8", mat).toImageMsg();
        pub->publish(*msg_out); // or: pub->publish(msg_out);
        }

    void DrawSonarLifecycleNode::sonarImageCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg){

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
        // DrawSonarLifecycleNode::cvBridgeAndPublish(msg, rotatedRect, rect_pub_);
        if (rect_pub_) cvBridgeAndPublish(msg, rotatedRect, rect_pub_);

        rectElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

        begin = std::chrono::steady_clock::now();

        cv::Mat sonarMat = sonar_drawer_.remapRectSonarImage(interface, rectMat);
        if (pub_) cvBridgeAndPublish(msg, sonarMat, pub_);

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


    CallbackReturn DrawSonarLifecycleNode::on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Configuring sonar draw lifecycle node");

        std::stringstream ss;
        ss << "configure parameters :\n"
        << "  max_range: " << max_range_ << "\n"
        << "  line_alpha: " << line_alpha_ << "\n"
        << "  range_spacing: " << range_spacing_ << "\n"
        << "  bearing_spacing: " << bearing_spacing_ << "\n";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        applyOverlayConfig(); 


        setColorMap(color_map_name_); 

        if (max_range_>0.0) {
            RCLCPP_INFO(get_logger(), "Only drawing to max range %.2f", max_range_);
          }

        sub_sonar_image_ = this->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>("sonar_image", 10, std::bind(&DrawSonarLifecycleNode::sonarImageCallback, this, std::placeholders::_1));

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
        std::bind(&DrawSonarLifecycleNode::on_param_change, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Configured sonar draw lifecycle node");

        return CallbackReturn::SUCCESS;
    }


    CallbackReturn DrawSonarLifecycleNode::on_activate(const rclcpp_lifecycle::State &)  {
        RCLCPP_INFO(get_logger(), "Activating sonar draw lifecycle node");

        if (old_pub_) old_pub_->on_activate();
        if (histogram_pub_) histogram_pub_->on_activate();
        if (rect_pub_) rect_pub_->on_activate();
        if (pub_) pub_->on_activate();
        if (osd_pub_) osd_pub_->on_activate();
        if (timing_pub_) timing_pub_->on_activate();

        if (sub_sonar_image_) {
          RCLCPP_INFO(get_logger(), "Subscriber for sonar_image activated.");
        } else {
            RCLCPP_WARN(get_logger(), "Subscriber for sonar_image failed to activate.");
        }


        return CallbackReturn::SUCCESS;
    }


    CallbackReturn DrawSonarLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)  {
        RCLCPP_INFO(get_logger(), "Deactivating sonar draw lifecycle node");

        if (old_pub_) old_pub_->on_deactivate();
        if (histogram_pub_) histogram_pub_->on_deactivate();
        if (rect_pub_) rect_pub_->on_deactivate();
        if (pub_) pub_->on_deactivate();
        if (osd_pub_) osd_pub_->on_deactivate();
        if (timing_pub_) timing_pub_->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DrawSonarLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)  {
        RCLCPP_INFO(get_logger(), "Cleaning up sonar draw lifecycle node");

        old_pub_.reset();
        histogram_pub_.reset();
        rect_pub_.reset();
        pub_.reset();
        osd_pub_.reset();
        timing_pub_.reset();

        sub_sonar_image_.reset();

        if (param_callback_handle_) {
          // remove callback by resetting handle
          param_callback_handle_.reset();
        }

        _colorMap.reset();

        return CallbackReturn::SUCCESS;
    }


    CallbackReturn DrawSonarLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &)  {
        RCLCPP_INFO(get_logger(), "Shutting down sonar draw lifecycle node");
        return CallbackReturn::SUCCESS;
    }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(draw_sonar::DrawSonarLifecycleNode)
