#ifndef DRAW_SONAR_LIFECYCLE_NODE_HPP_
#define DRAW_SONAR_LIFECYCLE_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>
#include <fstream>
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

//lifenode dependencies 
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace draw_sonar {
    class DrawSonarLifecycleNode : public rclcpp_lifecycle::LifecycleNode{

        public:
            explicit DrawSonarLifecycleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        protected:
            // lifecycle transistion state
            
            CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

            rcl_interfaces::msg::SetParametersResult
            on_param_change(const std::vector<rclcpp::Parameter> &parameters);  

            void setColorMap(const std::string &colorMapName);
            void applyOverlayConfig();
            void cvBridgeAndPublish(
            const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr &msg,
            const cv::Mat &mat,
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr &pub);

            void sonarImageCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg);

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

            // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_, rect_pub_, osd_pub_, old_pub_;
            // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
            // rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr histogram_pub_;
            
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr old_pub_;
            rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt32MultiArray>::SharedPtr histogram_pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr osd_pub_;
            rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr timing_pub_;


            std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_callback_handle_;


    };
}




#endif  //DRAW_SONAR_LIFECYCLE_NODE_HPP_