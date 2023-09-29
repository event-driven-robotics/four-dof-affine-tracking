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

#include "calib.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

void calibModule::CallBackFunc(int event, int x, int y, int flags, void* param)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        Point*p = (Point*)param;
        p->x = x;
        p->y = y;
    }
}

bool calibModule::configure(yarp::os::ResourceFinder& rf) {

    // options and parameters
    width = rf.check("w", Value(640)).asInt32();
    height = rf.check("h", Value(480)).asInt32();

    // module name
    setName((rf.check("name", Value("/calib")).asString()).c_str());

    if (!input_port.open(getName() + "/AE:i")){
        yInfo()<<"error";
        return false;
    }

    yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");

    eventsImage = cv::Mat::zeros(height, width, CV_8UC3);

    cv::namedWindow("calib", cv::WINDOW_NORMAL);
    cv::moveWindow("calib", 0,0);

    pts_monitor_plane.push_back(cv::Point(0,0)); 
    pts_monitor_plane.push_back(cv::Point(1920, 0)); 
    pts_monitor_plane.push_back(cv::Point(1920,1080)); 
    pts_monitor_plane.push_back(cv::Point(0,1080)); 

    Thread::start();

    return true;
}

void calibModule::run() {

    yInfo()<<"run module";
    while (Thread::isRunning()) {

        while (!input_port.isStopping()) {
            ev::info my_info = input_port.readAll(true);

            for (auto &v : input_port) {
                eventsImage.at<Vec3b>(v.y, v.x) = cv::Vec3b(255, 255, 255);
            }
        }
    }
}

double calibModule::getPeriod() {
    return 0.1;
}

bool calibModule::updateModule() {

    cv::circle(eventsImage, click_pos, 5, cv::Scalar(0, 0, 255), -1, 8, 0);

    if (click_pos.x!=click_pos_prev.x && click_pos.y!=click_pos_prev.y){
        click_pos_prev = click_pos;

        pts_image_plane.push_back(click_pos);
    }

    cv::namedWindow("calib", cv::WINDOW_NORMAL);
    setMouseCallback("calib", CallBackFunc, &click_pos);
    cv::imshow("calib", eventsImage);

    if (pts_image_plane.size() > 3){
        cv::Mat h = findHomography(pts_image_plane, pts_monitor_plane);   
        for (int i = 0; i < h.rows; i++) {
            for (int j = 0; j< h.cols; j++){
                std::cout<<h.at<double>(i,j)<<" ";
            }
            std::cout<<std::endl;
        }
        
    }

    cv::waitKey(1);

    // eventsImage = 0;

    return Thread::isRunning();
}

bool calibModule::interruptModule() {
    return Thread::stop();
}

void calibModule::onStop() {
    input_port.stop();
}

int main(int argc, char *argv[]) {
    /* initialize yarp network */
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("event-driven");
//    rf.setDefaultConfigFile( "tracker.ini" );
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    calibModule calib;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return calib.runModule(rf);
}
