# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.12)
project(libdrawsonar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wl,--no-as-needed")

# Set the output folder where your program will be created
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

# ###########################################
# The following folders will be included  #
# ###########################################
include_directories("${PROJECT_SOURCE_DIR}/include/")

find_package(spdlog)
find_package(OpenCV COMPONENTS core imgproc)

include_directories(${install_dir}/include/)

# #####################
# Add Execuatables  #
# #####################
link_directories(${Boost_LIBRARY_DIRS})

# Create Library
add_library(drawsonar SHARED ${drawsonar_SRCS})
set_target_properties(drawsonar PROPERTIES LIBRARY_OUTPUT_NAME drawsonar)
target_link_libraries(
    drawsonar
    PUBLIC spdlog::spdlog opencv_core opencv_imgproc opencv_highgui
)

# =============================================
# to allow find_package()
# =============================================
#
# The following is borrowed heavily from:
# https://github.com/RossHartley/invariant-ekf
# I am responsible for all mistakes
#
# the following case be used in an external project requiring drawsonar:
# ...
# find_package(drawsonar)
# include_directories(${drawsonar_INCLUDE_DIRS})
# ...

# NOTE: the following will support find_package for 1) local build (make) and 2) for installed files (make install)

# 1- local build

# Register the local build in case one doesn't use "make install"
export(PACKAGE drawsonar)

# Create variable for the local build tree
# set_target_properties(drawsonar PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
get_property(
    drawsonar_include_dirs
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTY INCLUDE_DIRECTORIES
)
get_property(
    drawsonar_library_dirs
    TARGET drawsonar
    PROPERTY LIBRARY_OUTPUT_DIRECTORY
)
get_property(drawsonar_libraries TARGET drawsonar PROPERTY LIBRARY_OUTPUT_NAME)

message("drawsonar_include_dirs: " ${drawsonar_include_dirs})
message("drawsonar_library_dirs: " ${drawsonar_library_dirs})
message("drawsonar_libraries: " ${drawsonar_libraries})

# Configure config file for local build tree
configure_file(
    cmake/drawsonarConfig.cmake.in
    "${PROJECT_BINARY_DIR}/drawsonarConfig.cmake"
    @ONLY
)

message("PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

# 2- installation build #

# Change the include location for the case of an install location
set(drawsonar_include_dirs ${CMAKE_INSTALL_PREFIX}/include ${EIGEN_INCLUDE_DIR})

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(
    cmake/drawsonarConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/drawsonarConfig.cmake"
    @ONLY
)

install(
    FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/drawsonarConfig.cmake"
    DESTINATION share/drawsonar/cmake
    COMPONENT dev
)
