# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(RGBtracker_realsense)

include(GNUInstallDirs)
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

find_package(OpenCV REQUIRED)                              
#include_directories(${OpenCV_INCLUDE_DIRS})

find_package(realsense2 REQUIRED)

find_package(YCM REQUIRED)
find_package(YARP COMPONENTS OS sig math dev idl_tools rosmsg REQUIRED)
find_package(event-driven REQUIRED)
#include_directories(../ ../../third-party/tclap/include ../../third-party/imgui)

yarp_add_idl(ROS_MSG ${CMAKE_SOURCE_DIR}/SharedData.msg)

#add_executable(RGBtracker_realsense ${CMAKE_SOURCE_DIR}/RGBtracker_realsense.cpp ../../librealsense/examples/example.hpp ../../librealsense/third-party/imgui/imgui.cpp ../../librealsense/third-party/imgui/imgui_draw.cpp ../../librealsense/third-party/imgui/imgui_impl_glfw.cpp)
add_executable(RGBtracker_realsense ${CMAKE_SOURCE_DIR}/RGBtracker_realsense.cpp ROScommunication.h affine.h)
target_link_libraries(RGBtracker_realsense PRIVATE 
                        ${OpenCV_LIBS}
                        YARP::YARP_OS
                        YARP::YARP_init
                        YARP::YARP_sig
                        YARP::YARP_dev
                        YARP::YARP_math
                        ev::event-driven
                        ${realsense2_LIBRARY} 
                        ${DEPENDENCIES})

install(TARGETS RGBtracker_realsense DESTINATION ${CMAKE_INSTALL_BINDIR})

# you can specify the CMAKE_INSTALL_BINDIR adding -DCMAKE_INSTALL_PREFIX=/path at the cmake .. command
install(TARGETS RGBtracker_realsense DESTINATION ${CMAKE_INSTALL_BINDIR})

