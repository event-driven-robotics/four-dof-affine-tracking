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
    double translation{0.5}, angle{0.05}, pscale{1.00001}, nscale{0.99991}, scaling{0.01};
    bool run{false};
    double dt_warpings{0}, dt_comparison{0}, dt_eros{0}, toc_count{0};
    std::string filename; 

    EROSfromYARP eros_handler;
    affineTransforms affine_handler;
    YarpToRos ros_publish;
    std::thread computation_thread;

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
    
        eros_k = rf.check("eros_k", Value(17)).asInt32();
        eros_d = rf.check("eros_d", Value(0.3)).asFloat64();
        period = rf.check("period", Value(0.01)).asFloat64();
        filename = rf.check("file", Value("/usr/local/src/affine2dtracking/thin_star.png")).asString(); 

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
            cv::normalize(affine_handler.mexican_template_64f, norm_mexican, 1, 0, cv::NORM_MINMAX);
            imshow("MEXICAN ROI", affine_handler.mexican_template_64f);
            imshow("TEMPLATE ROI", affine_handler.roi_template);
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

        return true;
    }

    void tracking_loop() {

        while (!isStopping()) {

            if (run){
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
                affine_handler.setEROS(eros_handler.eros.getSurface());
                // double toc_eros= Time::now();
                // yInfo()<<"eros";
                // double tic_comparison = Time::now();
                affine_handler.performComparisons();
                // double toc_comparison= Time::now();
                // yInfo()<<"comp";
                affine_handler.updateState();
                // yInfo()<<"state";

                ros_publish.publishTargetPos(img_size, affine_handler.initial_position.x+affine_handler.state[0], affine_handler.initial_position.y+affine_handler.state[1], affine_handler.state[2], affine_handler.state[3]); 

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