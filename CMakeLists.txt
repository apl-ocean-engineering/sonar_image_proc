cmake_minimum_required(VERSION 3.5)
project(sonar_image_proc)

if( FIPS_CONFIG AND NOT FIPS_IMPORT )
  cmake_minimum_required(VERSION 3.5)

  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup()
else()
  find_package(catkin QUIET)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find_package(Eigen3 REQUIRED )
find_package(Boost COMPONENTS filesystem system REQUIRED )  #n.b. program_options is only required for my version of Stereolabs "Recorder"

## Default OpenCV version
list(APPEND OPENCV_COMPONENTS core highgui imgproc )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)


if( FIPS_CONFIG )

  ## Set global include paths
  fips_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
   )

  # The local library
  fips_begin_module( sonar_image_proc )

    fips_src( lib *.cpp )
    fips_libs( ${Boost_LIBRARIES} ${OpenCV_LIBS} )

  fips_end_module()

  if( NOT FIPS_IMPORT )

    if( FIPS_UNITTESTS )
      gtest_begin( sonar_image_proc )
        fips_src( test/unit/ )
        fips_deps( sonar_image_proc )
      gtest_end()
    endif()

  endif()
  fips_finish()

else()

  find_package(catkin REQUIRED
    acoustic_msgs
    cv_bridge
    image_transport
    nodelet
    nodelet_topic_tools
    OpenCV
    pybind11_catkin
  )

  catkin_package(
      INCLUDE_DIRS include/ ros/include/
      LIBRARIES sonar_image_proc
      DEPENDS OpenCV
      CATKIN_DEPENDS pybind11_catkin
  )


  include_directories(
    include/
    ros/include/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
 )

  file(GLOB sonar_image_proc_SRCS
      lib/AbstractSonarInterface.cpp
   	  lib/ColorMaps.cpp
      lib/SonarDrawer.cpp
      lib/OldDrawSonar.cpp
  )

  # "true" library containing core functionality
  add_library(sonar_image_proc ${sonar_image_proc_SRCS})
  target_link_libraries(sonar_image_proc ${catkin_LIBRARIES})
  set_property(TARGET sonar_image_proc PROPERTY CXX_STANDARD 14)

  # Dynamically loadable library containing nodelets
  add_library(draw_sonar_nodelet ros/src/draw_sonar_nodelet.cpp )
  target_link_libraries(draw_sonar_nodelet ${catkin_LIBRARIES} sonar_image_proc )
  class_loader_hide_library_symbols(draw_sonar_nodelet)
  set_property(TARGET draw_sonar_nodelet PROPERTY CXX_STANDARD 14)

  add_executable(draw_sonar_node ros/src/draw_sonar_node.cpp)
  target_link_libraries(draw_sonar_node
     sonar_image_proc
     ${catkin_LIBRARIES})
  add_dependencies(draw_sonar_node ${catkin_EXPORTED_TARGETS})
  set_property(TARGET draw_sonar_node PROPERTY CXX_STANDARD 14)


  install(TARGETS draw_sonar_node sonar_image_proc
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#	install( DIRECTORY launch/
#	        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
#	install( DIRECTORY rqt_config/
#	        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt_config )

  install( FILES nodelet_plugins.xml
   		   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  ## Install headers
  install(DIRECTORY  include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
      PATTERN ".git" EXCLUDE)

  catkin_install_python(PROGRAMS
    scripts/sonar_pointcloud.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  ## Disable python wrapper for now ...
  #
  ## == Stuff related to the Python wrapper ==
  # pybind_add_module(py_draw_sonar SHARED
  # python/py_draw_sonar.cpp
  # python/ndarray_converter.cpp
  # lib/DrawSonar.cpp
  # lib/DataStructures.cpp
  # lib/ColorMaps.cpp
  # )
  # target_link_libraries( py_draw_sonar ${OpenCV_LIBS} draw_sonar )
  #
  # install(TARGETS py_draw_sonar
  # DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION} ) #${CATKIN_PACKAGE_PYTHON_DESTINATION})

  # catkin_python_setup()

endif()
