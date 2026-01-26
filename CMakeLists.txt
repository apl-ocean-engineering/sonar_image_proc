cmake_minimum_required(VERSION 3.14.4)
project(libdrawsonar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

# == Code common to all builds =======================================
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(drawsonar_SRCS
    lib/AbstractSonarInterface.cpp
    lib/ColorMaps.cpp
    lib/SonarDrawer.cpp
    lib/OldDrawSonar.cpp
    lib/HistogramGenerator.cpp
)

# ROS build, determine which version...
if(DEFINED ENV{ROS_VERSION})
    find_package(ros_environment)

    set(ROS_VERSION $ENV{ROS_VERSION})

    if(${ROS_VERSION} EQUAL 2)
        include(cmake/BuildROS2.cmake)
    elseif(${ROS_VERSION} EQUAL 1)
        include(cmake/BuildROS1.cmake)
    else()
        message(
            "Unsure what sort of build to do.  Not in a ROS1 or ROS2 environment"
        )
    endif()
else()
    message(NOTICE "!! Performing non-ROS build !!")
    include(cmake/BuildNonROS.cmake)
endif()
