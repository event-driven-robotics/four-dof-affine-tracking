#pragma once

#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>

using namespace yarp::os;

class EROSfromYARP
{
public:

    ev::window<ev::AE> input_port;
    ev::EROS eros;
    std::thread eros_worker;
    cv::Rect eros_update_roi;
    double dt_not_read_events{0};
    double tic{-1};
    double dur{0};
    int packet_events{0};

    void erosUpdate() 
    {
        while (!input_port.isStopping()) {
            ev::info my_info = input_port.readAll(true);
            tic = my_info.timestamp;
            dur = my_info.duration; 
            packet_events = my_info.count; 

            for(auto &v : input_port){
                if((v.x) > eros_update_roi.x && (v.x) < eros_update_roi.x + eros_update_roi.width && (v.y) > eros_update_roi.y && (v.y) < eros_update_roi.y + eros_update_roi.height)
                    eros.update(v.x, v.y);
            }

            dt_not_read_events = input_port.stats_unprocessed().duration;

        }
    }

public:
    bool start(cv::Size resolution, std::string sourcename, std::string portname, int k, double d)
    {
        eros.init(resolution.width, resolution.height, k, d);

        if (!input_port.open(portname))
            return false;

        yarp::os::Network::connect(sourcename, portname, "fast_tcp");

        eros_worker = std::thread([this]{erosUpdate();});
        return true;
    }

    void stop()
    {
        input_port.stop();
        eros_worker.join();
    }

};
