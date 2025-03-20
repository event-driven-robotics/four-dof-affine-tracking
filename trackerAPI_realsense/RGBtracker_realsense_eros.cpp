#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <yarp/os/all.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>

using namespace std;
using namespace cv;

class eventsReading{

    public: 
        ev::window<ev::AE> input_port;
        ev::EROS eros;
        std::thread eros_worker;
        double dt{0};

        bool start(cv::Size resolution, std::string sourcename, std::string portname, int k, double d)
        {
            eros.init(resolution.width, resolution.height, k, d);

            if (!input_port.open(portname))
                return false;

            yarp::os::Network::connect(sourcename, portname, "fast_tcp");

            eros_worker = std::thread([this]{erosUpdate();});
            return true;
        }


        void erosUpdate() 
        {
            while (!input_port.isStopping()) {
                ev::info my_info = input_port.readAll(true);
                static double t0 = my_info.timestamp; 
                dt = my_info.timestamp-t0; 
                for(auto &v : input_port){
                    eros.update(v.x, v.y);
                }
            }
        }



};


int main( int argc, char** argv ) {


    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    eventsReading erosCreation; 
    erosCreation.start(cv::Size(640,480), "/atis3/AE:o", "/eros_frames", 9, 0.5); 

    //Initialize
    // std::cout << "Waiting for the camera to be ready..." << std::flush;
    // rs2::pipeline pipe;
    // rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    // rs2::pipeline_profile pipeProfile = pipe.start(cfg);
    // rs2::pipeline_profile pipeProfile = pipe.start();
    // std::cout << " OK." << std::endl;

    // //Make image alignment
    // rs2::align align_to_color(RS2_STREAM_COLOR);

    // declares all required variables
    Rect roi;
    cv::Mat first_eros, cv_rgb_init, cv_rgb; 

    // create a tracker object
    Ptr<Tracker> tracker = TrackerKCF::create();

    std::chrono::duration<double> elapsed_seconds;

    // sleep(2.0);

    // Wait for new frames
    // rs2::frameset frameset = pipe.wait_for_frames();
    // Extract separate frames and colorize
    // auto color = frameset.get_color_frame();

    // Wrap frame data
    // const int w = color.as<rs2::video_frame>().get_width();
    // const int h = color.as<rs2::video_frame>().get_height();
    // cv::Mat cv_bgr(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
    // std::cout<<"dt="<<erosCreation.dt<<std::endl; 
    // cv::Rect first_roi = cv::Rect(260, 180, 120, 120); 

    // while (erosCreation.dt < 0.23){ //1.65 for 240, 0.24 for 1440
        // std::cout<<"dt="<<erosCreation.dt<<std::endl; 
        first_eros = erosCreation.eros.getSurface();
        // cv::rectangle(first_eros, first_roi, 255, 5);
        cv::cvtColor(first_eros, cv_rgb, cv::COLOR_GRAY2RGB);
        // imshow("first_eros", first_eros); 
    // }

    roi=selectROI("tracker",cv_rgb);

    
    // convert channel orders
    // cv::Mat cv_rgb = cv_bgr;

    // roi=selectROI("tracker",cv_rgb);


    //quit if ROI was not selected
    // if(first_roi.width==0 || first_roi.height==0)
    //     return 0;
    
    // initialize the tracker
    tracker->init(cv_rgb,roi);

    // cv_rgb = cv_rgb_init;

    auto start = std::chrono::system_clock::now();

    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    for ( ;; ){
        
        // get frame from the video
        // cap >> frame;
        // Wait for new frames
        // frameset = pipe.wait_for_frames();
        // // Align frames
        // frameset = align_to_color.process(frameset);
        // Extract separate frames and colorize
        // auto depth = frameset.get_depth_frame();
        // color = frameset.get_color_frame();        
        // Wrap frame data
        // const int w = depth.as<rs2::video_frame>().get_width();
        // const int h = depth.as<rs2::video_frame>().get_height();
        // cv::Mat cv_bgr(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
        // cv::Mat cv_depth(cv::Size(w, h), CV_16U, (void*) depth.get_data(), cv::Mat::AUTO_STEP);
        // cv_depth.convertTo(cv_depth, CV_32FC1, 0.001); // meters
        // convert channel orders
        // cv_rgb = cv_bgr;
        // cv::cvtColor(cv_bgr, cv_rgb, cv::COLOR_RGB2BGR);

        cv::Mat erosImg = erosCreation.eros.getSurface();

        cv::cvtColor(erosImg, cv_rgb, cv::COLOR_GRAY2RGB);

        // Mat imgHSV;

        // cvtColor(cv_rgb, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        // stop the program if no more images
        if(cv_rgb.rows==0 || cv_rgb.cols==0)
            break;
        
        // update the tracking result
        tracker->update(cv_rgb,roi);
        
        // draw the tracked object
        rectangle( cv_rgb, roi, Scalar( 255, 0, 0 ), 2, 1 );
        
        // show image with the tracked object
        imshow("tracker",cv_rgb);
        
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        cout << elapsed_seconds.count() << " seconds" << endl;
        start = std::chrono::system_clock::now();

        //quit on ESC button
        if(waitKey(1)==27)break;
    }

    return 0;

}