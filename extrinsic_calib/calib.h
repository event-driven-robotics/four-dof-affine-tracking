/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
 *   Author: luna.gava@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __CALIB__
#define __CALIB__

#pragma once

#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Time.h>
#include <yarp/rosmsg/geometry_msgs/Pose.h>

#include <event-driven/algs.h>

#include <iostream>
#include <mutex>
#include <cmath>
#include <tuple>
#include <numeric>
#include <vector>
#include <deque>
#include <map>
#include <eigen3/Eigen//Core>
#include <eigen3/Eigen/Eigenvalues>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

using yarp::os::Network;
using yarp::os::Node;
using yarp::os::Subscriber;

namespace {
    YARP_LOG_COMPONENT(LISTENER, "yarp.example.ros.listener")
}

class calibModule:public RFModule, public Thread{

private:

    int width, height;
    cv::Mat eventsImage;
    cv::Point click_pos{cv::Point(0,0)}, click_pos_prev{cv::Point(0,0)};
    std::vector<cv::Point2f> pts_image_plane, pts_monitor_plane;
    std::vector<double> monitor_positions, clicked_positions;
    int n_mapping;

    ev::window<ev::AE> input_port;

    static void CallBackFunc(int event, int x, int y, int flags, void* param);

protected:

public:

    yarp::sig::Vector ee_pos;

    // constructor
    calibModule(){}

    //the virtual functions that need to be overloaded

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual void onStop();
    virtual double getPeriod();
    virtual bool updateModule();

    void run();

};




#endif
//empty line