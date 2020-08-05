cmake_minimum_required(VERSION 3.5)
project(draw_sonar)

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
  fips_begin_module( draw_sonar )

    fips_src( lib *.cpp )
    fips_libs( ${Boost_LIBRARIES} ${OpenCV_LIBS} )

  fips_end_module()

  if( NOT FIPS_IMPORT )

    if( FIPS_UNITTESTS )
      gtest_begin( draw_sonar )
        fips_src( test/unit/ )
        fips_deps( draw_sonar )
      gtest_end()
    endif()

  endif()
  fips_finish()

else()

  find_package(catkin REQUIRED
    pybind11_catkin
    nodelet
    nodelet_topic_tools
    imaging_sonar_msgs
    image_transport
    cv_bridge
    OpenCV
  )

  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES draw_sonar
      DEPENDS OpenCV
      CATKIN_DEPENDS pybind11_catkin
  )


  include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
 )

  file(GLOB draw_sonar_SRCS
   	  lib/*.cpp
  )

  add_library(draw_sonar ${draw_sonar_SRCS} src_ros/draw_sonar_nodelet.cpp )
  target_link_libraries(draw_sonar ${catkin_LIBRARIES})
  class_loader_hide_library_symbols(draw_sonar)

  set_property(TARGET draw_sonar PROPERTY CXX_STANDARD 14)

  add_executable(draw_sonar_node src_ros/draw_sonar_node.cpp)
  target_link_libraries(draw_sonar_node
     draw_sonar
     ${catkin_LIBRARIES})
  add_dependencies(draw_sonar_node ${catkin_EXPORTED_TARGETS})

  set_property(TARGET draw_sonar_node PROPERTY CXX_STANDARD 14)


  install(TARGETS draw_sonar_node draw_sonar
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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


  ## == Stuff related to the Python wrapper ==
  pybind_add_module(py_draw_sonar SHARED
  python/py_draw_sonar.cpp
  python/ndarray_converter.cpp
  lib/DrawSonar.cpp
  lib/DataStructures.cpp
  lib/ColorMaps.cpp
  )
  # target_link_libraries( py_draw_sonar ${OpenCV_LIBS} draw_sonar )

  install(TARGETS py_draw_sonar
  DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION} ) #${CATKIN_PACKAGE_PYTHON_DESTINATION})

  catkin_python_setup()

endif()
