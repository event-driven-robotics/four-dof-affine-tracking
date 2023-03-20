cmake_minimum_required(VERSION 3.0)

project(shape_detector)


find_package(YARP COMPONENTS os sig REQUIRED)
find_package(event-driven REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} shape_detector.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              ${OpenCV_LIBRARIES}
                                              ev::event-driven)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})