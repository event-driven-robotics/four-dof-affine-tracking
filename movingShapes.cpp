#include <yarp/os/all.h>
using namespace yarp::os;

#include "eros.h"
#include<opencv2/imgproc.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>

class objectPos: public yarp::os::RFModule
{
private:
    int w, h, shape_w, shape_h;
    double period{0.1};
    std::string background_filename, shape_filename, output_filepath;

    cv::Point initial_position, new_position;

    cv::Mat shape_image, shift_right, shift_left, moved_shape, color_shape_image;
    int sum_tx{0}, sum_ty{0};
    double sum_rot{0.0}, sum_sc{0.0}, tot_sc{1.0};
    bool change_direction{false};
    int count{0}, count_pass_by_origin{0};
    cv::Mat background_image, resized_background, shape, local_background, current_background, cut, black_background, local_black_background, init_shape_image;
    cv::Rect mask, new_mask; 
    bool first_time{true}; 
    double perc{0.2};

    cv::Mat updateTrMat(double translation_x, double translation_y){

        // the state is an affine
        cv::Mat trMat = cv::Mat::zeros(2,3, CV_64F);

        trMat.at<double>(0,0) = 1; trMat.at<double>(0,1) = 0; trMat.at<double>(0,2) = translation_x;
        trMat.at<double>(1,0) = 0; trMat.at<double>(1,1) = 1; trMat.at<double>(1,2) = translation_y;

        return trMat;
    }


public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
        w = rf.check("w", Value(1920)).asInt64();
        h = rf.check("h", Value(1080)).asInt64();
        period = rf.check("period", Value(0.01)).asFloat64();
        shape_w = rf.check("shape_w", Value(300)).asInt64();
        shape_h = rf.check("shape_h", Value(300)).asInt64();
        perc = rf.check("perc", Value(0.2)).asFloat64();
        background_filename = rf.check("shape-file", Value("/usr/local/src/affine2dtracking/desk_background.jpg")).asString(); 
        shape_filename = rf.check("background-file", Value("/usr/local/src/affine2dtracking/white_star.png")).asString(); 
        output_filepath = rf.check("output-path", Value("/data/star_black_background/translation_x_1920x1080")).asString(); 

        // module name
        setName((rf.check("name", Value("/object_position")).asString()).c_str());

        background_image = cv::imread(background_filename, cv::IMREAD_COLOR);
        shape = cv::Mat::zeros(shape_h,shape_w, CV_8UC1);
        shape_image = cv::imread(shape_filename, 0);

        cv::Rect shape_rect = cv::boundingRect(shape_image); 

        cut = shape_image(shape_rect);
        cv::resize(cut, cut, cv::Size(perc*w,perc*w)); 

        cv::resize(shape_image, shape_image, cv::Size(perc*w,perc*w)); 

        shape_image.copyTo(init_shape_image);
        cv::cvtColor(shape_image, color_shape_image, cv::COLOR_GRAY2RGB);

        background_image = 0;
        cv::resize(background_image, resized_background, cv::Size(w,h)); 

        initial_position.x = resized_background.cols/2;
        initial_position.y = resized_background.rows/2;

        mask = cv::Rect(initial_position.x - shape_image.cols/2, initial_position.y - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        black_background = cv::Mat::zeros(resized_background.rows, resized_background.cols, CV_8U); 

        return true;
    }

    bool updateModule(){

    
        if (!change_direction){
            sum_tx += 1;
            // sum_ty += 1; 
            // sum_rot += 1; 
            // tot_sc = tot_sc * 1.01;
            // sum_sc += 0.5;

        }
        else{
            sum_tx -= 1;
            // sum_ty -= 1; 
            // sum_rot -= 1;
            // tot_sc = tot_sc*0.99;
            // sum_sc -= 0.5; 
        }
        // }


        new_position.x = initial_position.x + sum_tx; 
        new_position.y = initial_position.y + sum_ty; 


        black_background.copyTo(local_black_background); 

        cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2d(shape_image.cols/2, shape_image.rows/2), sum_rot, tot_sc);
        warpAffine(init_shape_image, shape_image, rot_mat, shape_image.size());
        // cv::resize(init_shape_image, shape_image, cv::Size(shape_image.cols+sum_sc, shape_image.rows+sum_sc), 0, 0, cv::INTER_LINEAR);

        new_mask = cv::Rect(new_position.x - shape_image.cols/2, new_position.y - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        shape_image.copyTo(local_black_background(new_mask)); 

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours( local_black_background, contours, hierarchy,cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

        resized_background.copyTo(current_background); 

        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        int idx = 0;
        for( ; idx >= 0; idx = hierarchy[idx][0] )
        {
            cv::Scalar color(255, 255, 255);
            drawContours( current_background, contours, idx, color, cv::FILLED, 8, hierarchy );
        }

        yInfo()<<sum_tx << sum_ty << sum_rot << sum_sc << tot_sc; 
    
        yInfo()<<w/2-shape_image.cols/2<<h/2-shape_image.rows/2;

        if(((sum_tx >= w/2-shape_image.cols/2) || (sum_ty >= h/2-shape_image.rows/2) || sum_rot > 180 || sum_sc > 20)&& !change_direction){ // (sum_sc) > 25
            change_direction = true;
            yInfo() << "direction changed";
            sum_sc = 0;
        }
        else if ((sum_tx <= -(w/2-shape_image.cols/2)|| (sum_ty <= -(h/2-shape_image.rows/2)) || sum_rot<-180 || sum_sc < -25) && change_direction){ // (sum_sc) < -25
            change_direction = false;
            yInfo() << "direction changed";
            sum_sc = 0;
        }

        imshow("test", current_background);
        cv::waitKey(1);

        if (count<10)
            imwrite(output_filepath+"/image-0000"+std::to_string(count)+".jpg", current_background);
        else if (count<100)
            imwrite(output_filepath+"/image-000"+std::to_string(count)+".jpg", current_background);
        else if (count<1000)
            imwrite(output_filepath+"/image-00"+std::to_string(count)+".jpg", current_background);
        else if (count<10000)
            imwrite(output_filepath+"/image-0"+std::to_string(count)+".jpg", current_background);
        else
            imwrite(output_filepath+"/image-"+std::to_string(count)+".jpg", current_background);

        count++;
        // yInfo()<<count<<new_position.x<<count_pass_by_origin; 

        // CODE TO STOP WHEN FIRST LOOP ENDED: uncomment for each axis of motion

        // if (new_position.x == w/2){
        //     if (count_pass_by_origin==1)
        //         return false; 
        //     count_pass_by_origin++; 
        //     yInfo()<<"hey"; 
        // }

        // if (new_position.y == h/2){
        //     if (count_pass_by_origin==1)
        //         return false; 
        //     count_pass_by_origin++; 
        //     yInfo()<<"hey"; 
        // }

        // if (sum_rot == 181 || sum_rot == 0){
        //     if (count_pass_by_origin==1)
        //         return false; 
        //     count_pass_by_origin++; 
        //     yInfo()<<"hey"; 
        // }

        // if (sum_sc == 0){
        //     if (count_pass_by_origin==2)
        //         return false; 
        //     count_pass_by_origin++; 
        //     yInfo()<<"hey"; 
        // }

        return true;
    }

    double getPeriod() override{
        return period;
    }

    bool interruptModule() override {
        return true;
    }

    bool close() override {

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
//    rf.setDefaultConfigFile( "tracker.ini" );
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    objectPos objectpos;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return objectpos.runModule(rf);
}