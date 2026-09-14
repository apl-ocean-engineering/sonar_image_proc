


#include "rclcpp/rclcpp.hpp"
// #include "draw_sonar_component.cpp"
#include "sonar_image_proc/draw_sonar_lifecycle_node.hpp"


int main(int argc, char **argv) {

  // libg3logger::G3Logger<ROSLogSink> log_worker("sonar_node");

  rclcpp::init(argc, argv);

  // auto node = std::make_shared<draw_sonar::DrawSonarComponent>(rclcpp::NodeOptions{});
  // rclcpp::spin(node);

  auto node = std::make_shared<draw_sonar::DrawSonarLifecycleNode>(rclcpp::NodeOptions{});

  rclcpp::spin(node->get_node_base_interface());


  rclcpp::shutdown();
  return 0;
}
