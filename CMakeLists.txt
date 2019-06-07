cmake_minimum_required(VERSION 3.5)
if( NOT FIPS_IMPORT )
	cmake_minimum_required(VERSION 3.5)

	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

	fips_setup(PROJECT serdp_common)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find_package(Eigen3 REQUIRED )
find_package(Boost COMPONENTS filesystem system REQUIRED )  #n.b. program_options is only required for my version of Stereolabs "Recorder"

## Default OpenCV version
list(APPEND OPENCV_COMPONENTS core highgui imgproc )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


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

fips_end_module()


if( NOT FIPS_IMPORT )

	if( FIPS_UNITTESTS )
			 gtest_begin( serdp_common )
			     fips_src( test/unit/ )
			     fips_deps( serdp_common )
			gtest_end()
	endif()

  fips_finish()
endif()
