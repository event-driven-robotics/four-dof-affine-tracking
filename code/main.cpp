#include <yarp/os/all.h>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace yarp::os;

#include "eros.h"
#include "affine.h"
#include "ROScommunication.h"

class affineTracking: public yarp::os::RFModule
{
private:

    cv::Size img_size;
    double period{0.01};
    double eros_k, eros_d;
    double tau_latency{0};
    double dT{0}; 
    double recording_duration, elapsed_time{0}; 
    double translation{1}, angle{0.5}, pscale{1.0001}, nscale{0.9999};
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};
    std::string filename; 
    std::deque< std::array<double, 19> > data_to_save;
    std::ofstream fs;
    std::vector<std::vector<double>> gt_values;
    bool gt_sending{false}; 
    int gt_index{0};
    double t_start_tracking{0};
    int counter{0};
    cv::Mat cv_bgr, cv_rgb;
    cv::Rect roi_rgb; 
    int count = 0;

    struct fake_latency{
        std::array<double, 4> state;
        double tstamp;
    };

    std::deque<fake_latency> fakeLat_queue;

    EROSfromYARP eros_handler;
    affineTransforms affine_handler;
    YarpToRos ros_publish;
    std::thread computation_thread;

public:

    std::vector<std::vector<double>> readFromCsv(std::string fname){

        std::vector<std::vector<double>> content;
        std::vector<double> times, state_vec;
        std::vector<std::vector<double>> gt_values;
        std::string line, word, ts, x, y, rot, sc;

        std::fstream file (fname, std::ios::in);
        if(file.is_open())
        {
            while(std::getline(file, line))
            {
                std::stringstream str(line);
    
                getline(str, ts, ',');
                times.push_back(std::stod(ts));
                getline(str, x, ',');
                state_vec.push_back(std::stod(x));
                getline(str, y, ',');
                state_vec.push_back(std::stod(y));
                getline(str, rot, ',');
                state_vec.push_back(std::stod(rot));
                getline(str, sc, '\n');
                state_vec.push_back(std::stod(sc));

                gt_values.push_back(state_vec); 
                state_vec.clear(); 
            }
        }
        else
            std::cout<<"Could not open the file\n";

        for(int i=0;i<times.size();i++)    
            std::cout<<i<<" "<<times[i]<<" "<<gt_values[i][0]<<" "<<gt_values[i][1]<<" "<<gt_values[i][2]<<" "<<gt_values[i][3]<<"\n";

        return gt_values; 
    }

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
    
        eros_k = rf.check("eros_k", Value(7)).asInt32();
        eros_d = rf.check("eros_d", Value(0.6)).asFloat64();
        if(!gt_sending)
            period = rf.check("period", Value(0.01)).asFloat64();
        else
            period = 0.001; 
        tau_latency=rf.check("tau", Value(0.0)).asFloat64();
        recording_duration = rf.check("rec_time", Value(10)).asFloat64();
        filename = rf.check("shape-file", Value("/usr/local/src/four-dof-affine-tracking/shapes/star.png")).asString(); 

        // module name
        setName((rf.check("name", Value("/shape-position")).asString()).c_str());

        yarp::os::Bottle& intrinsic_parameters = rf.findGroup("CAMERA_CALIBRATION");
        if (intrinsic_parameters.isNull()) {
            yError() << "Wrong .ini file or [CAMERA_CALIBRATION] not present. Deal breaker.";
            return false;
        }
        affine_handler.cam[affineTransforms::w] = intrinsic_parameters.find("w").asInt32();
        affine_handler.cam[affineTransforms::h] = intrinsic_parameters.find("h").asInt32();
        affine_handler.cam[affineTransforms::cx] = intrinsic_parameters.find("cx").asFloat32();
        affine_handler.cam[affineTransforms::cy] = intrinsic_parameters.find("cy").asFloat32();
        affine_handler.cam[affineTransforms::fx] = intrinsic_parameters.find("fx").asFloat32();
        affine_handler.cam[affineTransforms::fy] = intrinsic_parameters.find("fy").asFloat32();
        img_size = cv::Size(affine_handler.cam[affineTransforms::w], affine_handler.cam[affineTransforms::h]);

        yInfo() << "Camera Size:" << img_size.width << "x" << img_size.height;

        // cv::namedWindow("EROS FULL", cv::WINDOW_NORMAL);
        // cv::resizeWindow("EROS_FULL", img_size);
        // cv::moveWindow("EROS FULL", 0, 0);

        if (!eros_handler.start(img_size, "/atis3/AE:o", getName("/AE:i"), eros_k, eros_d)) {
            yError() << "could not open the YARP eros handler";
            return false;
        }
        yInfo()<<"eros started"; 

        if (rf.check("data-file")) {
            std::string ouputfilename = rf.find("data-file").asString();
            fs.open(ouputfilename);
            if (!fs.is_open()) {
                yError() << "Could not open output file" << filename;
                return false;
            }
        }

        if(gt_sending)
            gt_values = readFromCsv("/usr/local/src/affine2dtracking/gt_comb_400.csv");

        eros_handler.eros_update_roi=cv::Rect(0,0,640,480);

        affine_handler.init(translation, angle, pscale, nscale);
        affine_handler.initState();

        affine_handler.loadTemplate(img_size, filename);
        // affine_handler.createStaticTemplate(img_size, 30);
        // affine_handler.triangleTemplate(img_size); 

        yInfo()<<"shape template created";

        affine_handler.createAffines(translation, cv::Point(img_size.width/2,img_size.height/2), angle, pscale, nscale);

        affine_handler.create_maps(); 

        // yInfo()<<"maps created";
        ros_publish.initPublisher(); 

        computation_thread = std::thread([this]{tracking_loop();});

        if(!run) yInfo() << "WARNING: press G to start tracking (--run)";

        return true;
    }

    double getPeriod() override{
        return period;
    }
        
    bool updateModule() {
        cv::Mat norm_mexican;
        if (run){

            // if(gt_sending){
            //     yInfo()<<counter; 
            //     affine_handler.updateStateGT(gt_values, counter); 
            //     counter++;
            // }

            // if (gt_sending){
            //     ros_publish.publishTargetPos(img_size, gt_values[gt_index], img_size.height/2, 0, 1); 
            //     gt_index++; 
            // }
            // else{
                static double start_time = eros_handler.tic; 
                elapsed_time = eros_handler.tic - start_time;
                // elapsed_time = eros_handler.dt -0.2;
                // std::cout << std::fixed << std::setprecision(20) << eros_handler.tic<<" "<<start_time<<" "<<elapsed_time<<std::endl;

                // cv::Mat intersection_mat;
                // cv::bitwise_and(affine_handler.eros_filtered(affine_handler.roi_around_shape), affine_handler.rot_scaled_tr_template(affine_handler.roi_around_shape),intersection_mat);
                // cv::Mat union_mat = affine_handler.eros_filtered(affine_handler.roi_around_shape)+affine_handler.rot_scaled_tr_template(affine_handler.roi_around_shape);
                // double total_pixels = cv::countNonZero(union_mat); 

                // double matches = cv::countNonZero(intersection_mat);
                // double percentage = (100*matches/total_pixels);

                // yInfo()<<matches<<total_pixels<<percentage;
                // imshow("overlap", intersection_mat);
                // imshow("total", union_mat);

                //cv::normalize(affine_handler.mexican_template_64f, norm_mexican, 1, 0, cv::NORM_MINMAX);
                // imshow("MEXICAN ROI", affine_handler.mexican_template_64f+0.5);
                //imshow("TEMPLATE ROI", affine_handler.roi_template);
                // imshow("EROS ROI", affine_handler.eros_tracked);
                // cv::circle(eros_handler.eros.getSurface(), affine_handler.new_position, 2, 255, -1);
                // cv::rectangle(eros_handler.eros.getSurface(), affine_handler.roi_around_shape, 255,1,8,0);
                // imshow("EROS RESIZE", affine_handler.eros_resized);
                imshow("EROS FULL", affine_handler.eros_filtered+affine_handler.rot_scaled_tr_template);
                // imwrite("/usr/local/src/affine2dtracking/results/eros_images/"+std::to_string(count)+".jpg",eros_handler.eros.getSurface()); 

                // if (affine_handler.concat_affines.rows!=0 && affine_handler.concat_affines.cols!=0){
                //     cv::imshow("affine remap", affine_handler.concat_affines);
                // }

            // }
            
        }
        else{
            // cv::rectangle(eros_handler.eros.getSurface(), affine_handler.square, 255, 1, 8,0);
            imshow("EROS FULL", eros_handler.eros.getSurface()+affine_handler.initial_template);
        }

        int c = cv::waitKey(1);

        if (c == 32)
            affine_handler.initState();
        if (c == 'g')
            run = true;

        // yInfo()<<affine_handler.state[0]<<affine_handler.state[1]<<affine_handler.state[2]<<affine_handler.state[3];

        // if (toc_count){
        //     yInfo()<< (int)toc_count / (period) << "Hz";
        //     toc_count = 0;

        // }
        // yInfo() << dt_warpings<<dt_comparison<<dt_eros; 

        if(elapsed_time > recording_duration){
            yInfo()<<"recording duration reached"; 
            return false; 
        }

        // std::cout<<dT<<std::endl;
        // yInfo()<<dT<<affine_handler.roi_around_shape.width<<affine_handler.roi_around_shape.height;  

        count ++;

        return true;
    }

    void tracking_loop() {

        double tic = yarp::os::Time::now();
        while (!isStopping()) {

            // if ((eros_handler.dt)>0.2&& !run){
            //     run = true; 
            // }

            if (run){

                // yInfo()<<"run start";
                dT = yarp::os::Time::now() - tic;
                tic += dT;
                affine_handler.createDynamicTemplate();
                // yInfo()<<"dynamic template";
                affine_handler.updateAffines();
                affine_handler.setROI();
                // yInfo()<<"set roi";
                affine_handler.createMapWarpings();
                // yInfo()<<"maps created";
                // affine_handler.createWarpings(); 
                double eros_time_before = eros_handler.tic;
                affine_handler.setEROS(eros_handler.eros.getSurface());
                affine_handler.performComparisons();
                // yInfo()<<"comparison";
                // if (!gt_sending)
                affine_handler.updateStateAll();
                eros_handler.eros_update_roi = affine_handler.roi_around_shape;

                double eros_time_after = eros_handler.tic;
                double eros_diff_time = eros_time_after-eros_time_before;

                if(tau_latency>0){
                    fakeLat_queue.push_back({double(affine_handler.new_position.x), double(affine_handler.new_position.y), affine_handler.state[2], affine_handler.state[3], yarp::os::Time::now()});
                    std::array<double, 4> current_state;
                    bool found_pos_sent=false;
                    while(fakeLat_queue.size()>0 && (yarp::os::Time::now()-fakeLat_queue.front().tstamp)>tau_latency){
                        current_state = fakeLat_queue.front().state;
                        found_pos_sent = true;
                        fakeLat_queue.pop_front();
                    }
                    if (found_pos_sent){
                        ros_publish.publishTargetPos(img_size, current_state[0], current_state[1], current_state[2], current_state[3]);
                        data_to_save.push_back({elapsed_time, eros_handler.dur, double(eros_handler.packet_events), current_state[0], current_state[1], current_state[2], current_state[3], double(eros_handler.dt_not_read_events), eros_diff_time, dT, affine_handler.roi_around_shape.x, affine_handler.roi_around_shape.y, affine_handler.roi_around_shape.width, affine_handler.roi_around_shape.height, double(eros_handler.n_events_eros_update), double(eros_handler.eros_update_roi.x), double(eros_handler.eros_update_roi.y), double(eros_handler.eros_update_roi.width), double(eros_handler.eros_update_roi.height)});
                    }
                }else{
                    ros_publish.publishTargetPos(img_size, affine_handler.new_position.x, affine_handler.new_position.y, affine_handler.state[2], affine_handler.state[3]); 
                    if (fs.is_open() && eros_handler.tic > 0) {
                        data_to_save.push_back({elapsed_time, eros_handler.dur, double(eros_handler.packet_events), double(affine_handler.new_position.x), double(affine_handler.new_position.y), affine_handler.state[2], affine_handler.state[3], double(eros_handler.dt_not_read_events), eros_diff_time, dT, affine_handler.roi_around_shape.x, affine_handler.roi_around_shape.y, affine_handler.roi_around_shape.width, affine_handler.roi_around_shape.height, double(eros_handler.n_events_eros_update), double(eros_handler.eros_update_roi.x), double(eros_handler.eros_update_roi.y), double(eros_handler.eros_update_roi.width), double(eros_handler.eros_update_roi.height)});
                    }
                }
                // yInfo()<<"run end";
                
                // this->dt_warpings = toc_warpings - tic_warpings;
                // this->dt_comparison = toc_comparison - tic_comparison;
                // this->dt_eros = toc_eros - tic_eros;
                // this->toc_count++;

                // yInfo()<<"dt"<<dT;

            }

        }
    }

    bool interruptModule() override {
        return true;
    }

    bool close() override {

        yInfo() << "waiting for eros handler ... ";
        eros_handler.stop();
        yInfo() << "waiting for computation thread ... ";
        computation_thread.join();

        if(fs.is_open())
        {
            yInfo() << "Writing data";
            fs << "tau="<<tau_latency<<", tr="<<std::to_string(translation) <<", theta="<< std::to_string(angle) <<", pscale="<< std::to_string(pscale) << ", nscale="<< std::to_string(nscale) << ", scale_fact="<< std::to_string(affine_handler.sc_factor)<<", dynamic scale active="<< std::to_string(affine_handler.dynamic_scale)<<std::endl;
            fs << "proc_size="<<affine_handler.proc_size<<", mexican blur="<< std::to_string(affine_handler.blur) << ", eros decay="<< std::to_string(eros_d) <<", eros kernel="<<std::to_string(eros_k)<<", gaussian blur eros="<<std::to_string(affine_handler.gaussian_blur_eros)<<", median blur active="<<std::to_string(affine_handler.median_blur_active)<<", median blur eros="<<std::to_string(affine_handler.median_blur_eros)<<", eros buffer="<<std::to_string(affine_handler.buffer_width)<<std::endl;
            fs << "shape scale="<< std::to_string(affine_handler.template_scale)<<", shape blur="<< std::to_string(affine_handler.shape_blur_val)<<", canny 1="<< std::to_string(affine_handler.canny1)<<", canny 2="<< std::to_string(affine_handler.canny2)<<", sobel="<< std::to_string(affine_handler.sobel_value)<<", shape filename="<<filename<<std::endl; 
            for(auto i : data_to_save)
                fs << std::setprecision(20) << i[0] << " " << i[1] << " " << i[2] << " " << i[3] << " "<<i[4]<< " "<<i[5]<<" "<<i[6]<<" "<<i[7]<<" "<<i[8]<<" "<<i[9]<<" "<<i[10]<<" "<<i[11]<< " " << i[12] << " " << i[13] << " "<<i[14]<<" "<<i[15]<<" "<<i[16]<<" "<<i[17]<<" "<<i[18]<<std::endl;
            fs.close();
            yInfo() << "Finished Writing data";
        }

        return true;
    }
};

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
    rf.setDefaultConfigFile("/usr/local/src/four-dof-affine-tracking/code/config.ini");
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    affineTracking affinetracking;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return affinetracking.runModule(rf);
}
