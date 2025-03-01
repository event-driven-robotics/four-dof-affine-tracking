# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(shape_position)

if(NOT CMAKE_BUILD_TYPE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

include(GNUInstallDirs)

# mandatory use of these packages
find_package(YCM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YARP COMPONENTS OS sig math dev idl_tools rosmsg REQUIRED)
find_package(event-driven REQUIRED)

yarp_add_idl(ROS_MSG SharedData.msg)

# add_executable(${PROJECT_NAME} analysis.cpp eros.h affine.h)
add_executable(${PROJECT_NAME} main.cpp eros.h affine.h ROScommunication.h)
# add_executable(${PROJECT_NAME} main.cpp eros.h affine.h ROScommunication.h ${ROS_MSG})
add_executable(stimulus movingShapes.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev
        YARP::YARP_math
        ev::event-driven
        ${OpenCV_LIBRARIES})

target_link_libraries(stimulus PRIVATE YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev
        YARP::YARP_math
        ev::event-driven
        ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

