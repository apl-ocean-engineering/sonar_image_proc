cmake_minimum_required(VERSION 3.5)

if( FIPS_CONFIG AND NOT FIPS_IMPORT )
	cmake_minimum_required(VERSION 3.5)

	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

	fips_setup(PROJECT serdp_common)

else()
  find_package(catkin QUIET)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find_package(Eigen3 REQUIRED )
find_package(Boost COMPONENTS filesystem system REQUIRED )  #n.b. program_options is only required for my version of Stereolabs "Recorder"

## Default OpenCV version
list(APPEND OPENCV_COMPONENTS core highgui imgproc )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


if( FIPS_CONFIG )
  ## Set global include paths
  fips_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
   )

  # The local library
  fips_begin_module( serdp_common )

    fips_src( lib *.cpp )
    fips_deps( oculus )
    fips_libs( ${Boost_LIBRARIES} ${OpenCV_LIBS} )

  fips_end_module()


	if( NOT FIPS_IMPORT )

    if( FIPS_UNITTESTS )
      gtest_begin( serdp_common )
        fips_src( test/unit/ )
        fips_deps( serdp_common )
      gtest_end()
    endif()

	endif()
  fips_finish()

else()
    project(serdp_common)

    find_package(catkin REQUIRED
      liboculus REQUIRED
      g3log_catkin
      active_object
    )

    catkin_package(
    	  #CATKIN_DEPENDS liboculus
        INCLUDE_DIRS include
        LIBRARIES serdp_LIB
    )


    include_directories(
      ${catkin_INCLUDE_DIRS}
      include/
      ${liboculus_INCLUDE_DIRS}
   )

    file(GLOB serdp_common_SRCS
     	  lib/*.cpp
    )

    add_library(serdp_LIB ${serdp_common_SRCS})

    install(TARGETS serdp_LIB
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


    ## Install headers
    install(DIRECTORY  include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".git" EXCLUDE)

# in your other catkin projects the libraries will be available as:
# ${serdp_common_INCLUDE_DIRS}
# ${serdp_common_LIBRARIES}

endif()
