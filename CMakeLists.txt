cmake_minimum_required(VERSION 3.10)
project(sonar_image_proc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS 2 and third-party dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(marine_acoustic_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# --- Library sources ---
set(SONAR_IMAGE_PROC_SRCS
  lib/AbstractSonarInterface.cpp
  lib/ColorMaps.cpp
  lib/SonarDrawer.cpp
  lib/OldDrawSonar.cpp
  lib/HistogramGenerator.cpp
)

# --- Composable node library ---
add_library(sonar_image_proc_components SHARED
  ${SONAR_IMAGE_PROC_SRCS}
  ros/src/draw_sonar_component.cpp
  ros/src/draw_sonar_lifecycle_node.cpp
  # ros/src/sonar_postprocessor_nodelet.cpp
)

target_include_directories(sonar_image_proc_components
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sonar_image_proc_components
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  OpenCV
  marine_acoustic_msgs
  lifecycle_msgs
  rclcpp_lifecycle
)
rclcpp_components_register_nodes(sonar_image_proc_components
  PLUGIN "draw_sonar::DrawSonarComponent"
  "draw_sonar::DrawSonarLifecycleNode"
  # PLUGIN "sonar_image_proc::SonarPostprocessor"
)

install(TARGETS sonar_image_proc_components
  EXPORT export_sonar_image_proc_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_sonar_image_proc_component)

# --- Standalone node ---
add_executable(draw_sonar_node ros/src/draw_sonar_node.cpp)
target_include_directories(draw_sonar_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ros/include
)
ament_target_dependencies(draw_sonar_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  OpenCV
  marine_acoustic_msgs
  lifecycle_msgs
  rclcpp_lifecycle
)
target_link_libraries(draw_sonar_node sonar_image_proc_components)
install(TARGETS draw_sonar_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Install headers from both include trees ---
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(
    DIRECTORY ros/config/
    DESTINATION share/${PROJECT_NAME}/config
)
install(TARGETS draw_sonar_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# --- Install python scripts if any ---
install(PROGRAMS
  scripts/sonar_pointcloud.py
  scripts/sonar_fov.py
  DESTINATION lib/${PROJECT_NAME}
)
set_target_properties(draw_sonar_node PROPERTIES OUTPUT_NAME draw_sonar_node)
ament_package()



