#include <yarp/os/all.h>
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
    double recording_duration, elapsed_time{0}; 
    double translation{2.0}, angle{0.8}, pscale{1.05}, nscale{0.95}, scaling{0.01};
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};
    std::string filename; 
    std::deque< std::array<double, 14> > data_to_save;
    std::ofstream fs;

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

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
    
        eros_k = rf.check("eros_k", Value(9)).asInt32();
        eros_d = rf.check("eros_d", Value(0.5)).asFloat64();
        period = rf.check("period", Value(0.01)).asFloat64();
        tau_latency=rf.check("tau", Value(0.0)).asFloat64();
        recording_duration = rf.check("rec_time", Value(10000)).asFloat64();
        filename = rf.check("shape-file", Value("/usr/local/src/affine2dtracking/shapes/thin_star.png")).asString(); 

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

        eros_handler.setEROSupdateROI(cv::Rect(0,0,640,480));

        affine_handler.init(translation, angle, pscale, nscale, scaling);
        affine_handler.initState();

        affine_handler.loadTemplate(img_size, filename);
        // affine_handler.createStaticTemplate(img_size, 30);
        // affine_handler.squareTemplate(img_size); 

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
            static double start_time = eros_handler.tic; 
            elapsed_time = eros_handler.tic - start_time;

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
            //imshow("MEXICAN ROI", affine_handler.mexican_template_64f+0.5);
            //imshow("TEMPLATE ROI", affine_handler.roi_template);
            imshow("EROS ROI", affine_handler.eros_tracked);
            // cv::circle(eros_handler.eros.getSurface(), affine_handler.new_position, 2, 255, -1);
            // cv::rectangle(eros_handler.eros.getSurface(), affine_handler.roi_around_shape, 255,1,8,0);
            imshow("EROS FULL", affine_handler.eros_filtered+affine_handler.rot_scaled_tr_template);
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
        //     toc_count = dt = 0;

        // }
        // yInfo() << dt_warpings<<dt_comparison<<dt_eros; 

        if(elapsed_time > recording_duration){
            yInfo()<<"recording duration reached"; 
            return false; 
        }

        return true;
    }

    void tracking_loop() {

        double tic = yarp::os::Time::now();
        while (!isStopping()) {

            if (run){

                double dT = yarp::os::Time::now() - tic;
                tic += dT;
                affine_handler.createDynamicTemplate();
                // yInfo()<<"template";
                affine_handler.setROI();
                // yInfo()<<"roi";
                affine_handler.updateAffines();
                // yInfo()<<"update affine";
                affine_handler.make_template();
                // yInfo()<<"mexican";
                // double tic_warpings = Time::now();
                affine_handler.createWarpings();
                // double toc_warpings = Time::now();
                // affine_handler.createMapWarpings(); 
                // yInfo()<<"remap";
                // double tic_eros = Time::now();
                double eros_time_before = eros_handler.tic;
                affine_handler.setEROS(eros_handler.eros.getSurface());
                // double toc_eros= Time::now();
                // yInfo()<<"eros";
                // double tic_comparison = Time::now();
                affine_handler.performComparisons();
                // double toc_comparison= Time::now();
                // yInfo()<<"comp";
                affine_handler.updateStateAll();
                // yInfo()<<"state";                
                eros_handler.setEROSupdateROI(affine_handler.roi_around_shape);
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
                        data_to_save.push_back({elapsed_time, eros_handler.dur, double(eros_handler.packet_events), current_state[0], current_state[1], current_state[2], current_state[3], double(eros_handler.dt_not_read_events), eros_diff_time, dT, affine_handler.roi_around_shape.x, affine_handler.roi_around_shape.y, affine_handler.roi_around_shape.width, affine_handler.roi_around_shape.height});
                    }

                }else{
                    ros_publish.publishTargetPos(img_size, affine_handler.new_position.x, affine_handler.new_position.y, affine_handler.state[2], affine_handler.state[3]); 
                    if (fs.is_open() && eros_handler.tic > 0) {
                        data_to_save.push_back({elapsed_time, eros_handler.dur, double(eros_handler.packet_events), double(affine_handler.new_position.x), double(affine_handler.new_position.y), affine_handler.state[2], affine_handler.state[3], double(eros_handler.dt_not_read_events), eros_diff_time, dT, affine_handler.roi_around_shape.x, affine_handler.roi_around_shape.y, affine_handler.roi_around_shape.width, affine_handler.roi_around_shape.height});
                    }
                }
                
                // this->dt_warpings = toc_warpings - tic_warpings;
                // this->dt_comparison = toc_comparison - tic_comparison;
                // this->dt_eros = toc_eros - tic_eros;
                // this->toc_count++;

                // yInfo()<<"running";

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
            fs << "tr="<<std::to_string(translation) <<", theta="<< std::to_string(angle) <<", pscale="<< std::to_string(pscale) << ", nscale="<< std::to_string(nscale) <<std::endl;
            fs << "mexican blur="<< std::to_string(affine_handler.blur) << ", eros decay="<< std::to_string(eros_d) <<", eros kernel="<<std::to_string(eros_k)<<", gaussian blur eros="<<std::to_string(affine_handler.gaussian_blur_eros)<<", shape scale="<< std::to_string(affine_handler.template_scale)<<", shape filename="<<filename<<std::endl; 
            for(auto i : data_to_save)
                fs << std::setprecision(20) << i[0] << " " << i[1] << " " << i[2] << " " << i[3] << " "<<i[4]<< " "<<i[5]<<" "<<i[6]<<" "<<i[7]<<" "<<i[8]<<" "<<i[9]<<" "<<i[10]<<" "<<i[11]<< " " << i[12] << " " << i[13] << " "<<i[14]<<std::endl;
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
    rf.setDefaultConfigFile("/usr/local/src/affine2dtracking/config.ini");
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    affineTracking affinetracking;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return affinetracking.runModule(rf);
}