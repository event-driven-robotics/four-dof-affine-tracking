# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(calib)

find_package(YCM REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(YARP COMPONENTS OS sig math dev REQUIRED)
find_package(event-driven REQUIRED)

include(GNUInstallDirs)
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
        LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
        INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
        USE_LINK_PATH)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.h ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev
        YARP::YARP_math
        ev::event-driven
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
#file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/app_vAirHockeyDemo.xml)
#yarp_install(FILES ${scripts} DESTINATION ${EVENT-DRIVEN_APPLICATIONS_INSTALL_DIR})
