cmake_minimum_required(VERSION 3.0)

project(interpolate_trajectory)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} interpolate_trajectory.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBRARIES})

#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
