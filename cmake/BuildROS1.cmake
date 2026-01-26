# Catkin/ROS1 section =====

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES libdrawsonar
)

add_library(libdrawsonar ${drawsonar_SRCS})

include_directories(libdrawsonar include ${catkin_INCLUDE_DIRS})

target_link_libraries(libdrawsonar ${catkin_LIBRARIES})

install(
    TARGETS libdrawsonar
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
    PATTERN ".git" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
    add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    include_directories(test/data/)

    file(GLOB drawsonar_test_SRCS test/unit/*cpp)

    catkin_add_gtest(drawsonar_test ${drawsonar_test_SRCS})

    target_link_libraries(
        drawsonar_test
        ${catkin_LIBRARIES}
        libdrawsonar
        Boost::system
    )
endif()
