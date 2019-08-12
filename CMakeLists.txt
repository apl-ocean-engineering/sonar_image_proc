cmake_minimum_required(VERSION 3.5)

find_package(catkin QUIET)
if( NOT FIPS_IMPORT )
    if (NOT catkin_FOUND)
	cmake_minimum_required(VERSION 3.5)

	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

	fips_setup(PROJECT serdp_common)
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find_package(Eigen3 REQUIRED )
find_package(Boost COMPONENTS filesystem system REQUIRED )  #n.b. program_options is only required for my version of Stereolabs "Recorder"

## Default OpenCV version
list(APPEND OPENCV_COMPONENTS core highgui imgproc )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


if (FIPS_IMPORT)
## Set global include paths
    fips_include_directories(
      ${CMAKE_SOURCE_DIR}/include
      ${OpenCV_INCLUDE_DIRS}
     )
	# The local library
	fips_begin_module( serdp_common )

	   fips_src( lib *.cpp )

	   fips_deps( oculus )

           fips_libs(  ${Boost_LIBRARIES} )

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
    find_package(catkin REQUIRED COMPONENTS
      liboculus REQUIRED
    )

    include_directories(
      ${catkin_INCLUDE_DIRS}
      include/
      ${liboculus_INCLUDE_DIRS}
   )

    catkin_package(
    	  #CATKIN_DEPENDS liboculus
        INCLUDE_DIRS include
    )

    file(GLOB serdp_common_SRCS
     	  lib/*.cpp
    )

    #add_library(serdp ${serdp_common_SRCS})

    install(FILES ${serdp_common_SRCS}
            DESTINATION lib)


    ## Install headers
    install(DIRECTORY  include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".git" EXCLUDE)

# in your other catkin projects the g2o libraries will be available as:
# ${serdp_common_INCLUDE_DIRS}
# ${serdp_common_LIBRARIES}

endif()
