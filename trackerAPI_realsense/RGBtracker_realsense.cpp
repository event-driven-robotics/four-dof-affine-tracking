#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <yarp/os/all.h>
#include "ROScommunication.h"
#include "affine.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <deque>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

    //Initialize
    std::cout << "Waiting for the camera to be ready..." << std::flush;
    rs2::pipeline pipe;
    rs2::config cfg;
    bool camera{true}; 
    int algorithm{2}; // 1 -> kcf , 2 -> affine 
    bool run{false}; 
    std::deque< std::array<double, 3> > data_to_save;
    std::ofstream fs;
    static cv::Mat first_rgb; 
    int count{0}; 

    fs.open("rgb_star_trans_x_1440hz.txt");
    if (!fs.is_open()) {
        yError() << "Could not open output file";
        return false;
    }

    double translation{6}, angle{1}, pscale{1.01}, nscale{0.99};
    affineTransforms affine_handler;
    YarpToRos ros_publish;
    std::string template_filename = "/home/luna/Downloads/workbook_yvonne-anne-gabrielle-vullers/code/star.png"; 
    rs2::colorizer color_map;

    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    if (camera)
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    else
        cfg.enable_device_from_file("/home/luna/shared/data/moving_background/rgb-datasets/rgb_star_trans_x_480.bag");

    rs2::pipeline_profile pipeProfile = pipe.start(cfg);
    // // rs2::pipeline_profile pipeProfile = pipe.start();
    // std::cout << " OK." << std::endl;

    //rs2::device device = pipe.get_active_profile().get_device();

    //rs2::playback playback = device.as<rs2::playback>();
    //pipe.stop(); // Stop streaming with default configuration
    //pipe = std::make_shared<rs2::pipeline>();
    //rs2::config cfg;
   
    //pipe.start(cfg); //File will be opened in read mode at this point
    //device = pipe->get_active_profile().get_device();  

    // //Make image alignment
    // rs2::align align_to_color(RS2_STREAM_COLOR);

    // declares all required variables
    cv::Rect roi;

    // create a tracker object
    Ptr<Tracker> tracker = TrackerKCF::create();

    std::chrono::duration<double> elapsed_seconds;

    sleep(2.0);

    // Wait for new frames
    rs2::frameset frameset = pipe.wait_for_frames();
    // Extract separate frames and colorize
    auto color = frameset.get_color_frame();

    // Wrap frame data
    const int w = color.as<rs2::video_frame>().get_width();
    const int h = color.as<rs2::video_frame>().get_height();
    cv::Mat cv_bgr(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
    // convert channel orders
    cv::Mat cv_rgb = cv_bgr;
    cv::cvtColor(cv_bgr, cv_rgb, cv::COLOR_RGB2BGR);

    roi=selectROI("tracker",cv_rgb);

    //quit if ROI was not selected
    if(roi.width==0 || roi.height==0)
        return 0;
    
    // initialize the tracker
    if (algorithm == 1)
        tracker->init(cv_rgb,roi);
    else{
        affine_handler.init(translation, angle, pscale, nscale);
        // yInfo()<<"INIT";
        affine_handler.initState();
        // yInfo()<<"INIT state";
        affine_handler.loadTemplate(cv::Size(w,h), template_filename);
        // yInfo()<<"TEMPLATE loaded";
        affine_handler.createAffines(translation, cv::Point(w/2,h/2), angle, pscale, nscale);
        // yInfo()<<"AFFINES created";
        affine_handler.create_maps(); 
        // yInfo()<<"MAPS created";
    }


    ros_publish.initPublisher(); 

    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    for ( ;; ){
        
        // get frame from the video
        // cap >> frame;
        // Wait for new frames
        frameset = pipe.wait_for_frames();
        // Align frames
        // frameset = align_to_color.process(frameset);
        // Extract separate frames and colorize
        // auto depth = frameset.get_depth_frame();
        color = frameset.get_color_frame();        
        // Wrap frame data
        // const int w = depth.as<rs2::video_frame>().get_width();
        // const int h = depth.as<rs2::video_frame>().get_height();
        cv::Mat cv_bgr(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
        // cv::Mat cv_depth(cv::Size(w, h), CV_16U, (void*) depth.get_data(), cv::Mat::AUTO_STEP);
        // cv_depth.convertTo(cv_depth, CV_32FC1, 0.001); // meters
        // convert channel orders
        cv_rgb = cv_bgr;
        cv::cvtColor(cv_bgr, cv_rgb, cv::COLOR_RGB2BGR);

        // Mat imgHSV;

        // cvtColor(cv_rgb, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        // stop the program if no more images
        if(cv_rgb.rows==0 || cv_rgb.cols==0)
            break;
        
        // imwrite("/home/luna/code/workbook_marco-monforte/trackerAPI_realsense/build/motion_blur/"+std::to_string(count)+".jpg", cv_rgb); 


        // update the tracking result
        if (algorithm == 1){
            tracker->update(cv_rgb,roi);        
            // draw the tracked object
            rectangle( cv_rgb, roi, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else{
            if(run == true){
                // first_rgb = cv_rgb;  
                // auto start = std::chrono::system_clock::now();
                static double t0 = yarp::os::Time::now(); 
                affine_handler.createDynamicTemplate();
                // yInfo()<<"DYN TEMPLATE created";
                affine_handler.updateAffines();
                // yInfo()<<"AFFINES updated";
                affine_handler.setROI();
                // yInfo()<<"set ROI";
                affine_handler.createMapWarpings();
                // yInfo()<<"MAP WARPINGS created";
                affine_handler.setRGB(cv_rgb, roi); 
                // yInfo()<<"set RGB";
                affine_handler.performComparisons();
                // yInfo()<<"comparison";
                affine_handler.updateStateAll();
                // yInfo()<<affine_handler.new_position.x<<affine_handler.new_position.y<<affine_handler.state[2]<<affine_handler.state[3];
                rectangle( cv_rgb, cv::Rect(affine_handler.state[0]+w/2-roi.width/2,affine_handler.state[1]+h/2-roi.height/2, roi.width, roi.height), Scalar( 255, 0, 0 ), 2, 1 );
                roi = cv::Rect(affine_handler.state[0]+w/2-roi.width/2,affine_handler.state[1]+h/2-roi.height/2, roi.width, roi.height); 
                double delta = yarp::os::Time::now() - t0;
                // data_to_save.push_back({double(delta), double(affine_handler.state[0]+w/2), double(affine_handler.state[1]+h/2) });

                // yInfo()<<delta; 
                // auto end = std::chrono::system_clock::now();
                // std::chrono::duration<double> elapsed_seconds = end - start;
                // cout << elapsed_seconds.count() << " seconds" << endl;
                // start = std::chrono::system_clock::now();
            }
        }
        
        ros_publish.publishTargetPos(cv::Size(640,480), affine_handler.new_position.x, affine_handler.new_position.y, affine_handler.state[2], affine_handler.state[3]); 

        // show image with the tracked object
        imshow("tracker",cv_rgb);
        int c = cv::waitKey(1);
        // if (c =='h'){
        //     imwrite("rgb_star_trans_x_1440hz.jpg", first_rgb); 
        //     if(fs.is_open())
        //     {
        //         yInfo() << "Writing data";
        //         for(auto i : data_to_save)
        //             fs << std::setprecision(5) << i[0] << " " << i[1] << " " << i[2] << std::endl;
        //         fs.close();
        //         yInfo() << "Finished Writing data";
        //     }
        //     return 0; 
        // }
        if (c == 32)
            affine_handler.initState();
        if (c == 'g')
            run = true;

        count++; 

    }

    return 0;

}