# == ament/ROS2 section =================================

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_library(drawsonar SHARED ${drawsonar_SRCS})
target_link_libraries(drawsonar PUBLIC opencv_core)

target_include_directories(
    drawsonar
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/>"
)

install(
    TARGETS drawsonar
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
    PATTERN ".git" EXCLUDE
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(drawsonar)

ament_package()
