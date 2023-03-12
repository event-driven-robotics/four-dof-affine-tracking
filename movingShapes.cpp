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
    int blur{21};

    ev::window<ev::AE> input_port;

    ev::EROS eros;
    int eros_k;
    double eros_d;
    cv::Mat eros_filtered, eros_detected, eros_detected_roi, eros_tracked, eros_tracked_roi, eros_tracked_32f, eros_tracked_64f;

    std::thread affine_thread;
    std::ofstream fs;

    typedef struct affine_struct
    {
        cv::Mat A;
        cv::Mat warped_img;
        double score;

    } affine_bundle;

    std::array<affine_bundle, 9> affine_info;
    double translation{2}, angle{1.0}, pscale{1.01}, nscale{0.99};
    std::array<double, 4> state;
    std::vector<cv::Mat> affines_vector;
    std::vector<double> scores_vector;
    cv::Mat initial_template, roi_template, roi_template_64f, mexican_template, mexican_template_64f;
    cv::Point initial_position;
    cv::Point new_position;

    cv::Rect square;
    cv::Rect roi_around_shape;

    cv::Mat shape_image, shift_right, shift_left, moved_shape, color_shape_image;
    int sum_tx{0}, sum_ty{0};
    double sum_rot{0.0}, sum_sc{0.0}, tot_sc{1.0};
    bool change_direction{false};
    cv::VideoWriter vid_writer;
    int count{0}, count_pass_by_origin{0};
    cv::Mat background_image, resized_background, shape, local_background, current_background, cut, black_background, local_black_background, init_shape_image;
    cv::Rect mask, new_mask; 
    bool first_time{true}; 
    double perc{0.2};

    void squareTemplate(){
        square = cv::Rect((1920-300)/2,(1080-300)/2, 300, 300);
        cv::rectangle(initial_template, square, cv::Scalar(255),1,8,0);
        cv::rectangle(initial_template, square, cv::Scalar(0),-(square.width-1),8,0);

        initial_position.x = square.x + square.width/2;
        initial_position.y = square.y + square.height/2;
    }

    void starTemplate(){
        cv::line(initial_template, cv::Point(327,172), cv::Point(336,204), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(336,204), cv::Point(367,208), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(367,208), cv::Point(341,226), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(341,226), cv::Point(348,258), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(348,258), cv::Point(324,237), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(324,237), cv::Point(296,256), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(296,256), cv::Point(308,225), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(308,225), cv::Point(283,203), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(283,203), cv::Point(316,202), cv::Scalar(255),1,8,0);
        cv::line(initial_template, cv::Point(316,202), cv::Point(327,172), cv::Scalar(255),1,8,0);

        initial_position.x = 320;
        initial_position.y = 240;
    }

    void initializeAffines(){
        for(auto &affine : affine_info) {
            affine.A = cv::Mat::zeros(2,3, CV_64F);
        }
    }

    void createAffines(double translation, cv::Point2f center, double angle, double pscale, double nscale){
        initializeAffines();

        affine_info[0].A.at<double>(0,0) = 1;
        affine_info[0].A.at<double>(0,2) = translation;
        affine_info[0].A.at<double>(1,1) = 1;
        affine_info[0].A.at<double>(1,2) = 0;

        affine_info[1].A.at<double>(0,0) = 1;
        affine_info[1].A.at<double>(0,2) = -translation;
        affine_info[1].A.at<double>(1,1) = 1;
        affine_info[1].A.at<double>(1,2) = 0;

        affine_info[2].A.at<double>(0,0) = 1;
        affine_info[2].A.at<double>(0,2) = 0;
        affine_info[2].A.at<double>(1,1) = 1;
        affine_info[2].A.at<double>(1,2) = translation;

        affine_info[3].A.at<double>(0,0) = 1;
        affine_info[3].A.at<double>(0,2) = 0;
        affine_info[3].A.at<double>(1,1) = 1;
        affine_info[3].A.at<double>(1,2) = -translation;

        affine_info[4].A = cv::getRotationMatrix2D(center, angle, 1);
        affine_info[5].A = cv::getRotationMatrix2D(center, -angle, 1);

        affine_info[6].A = cv::getRotationMatrix2D(center, 0, pscale);
        affine_info[7].A = cv::getRotationMatrix2D(center, 0, nscale);

        affine_info[8].A.at<double>(0,0) = 1;
        affine_info[8].A.at<double>(1,1) = 1;
    }

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
        w = rf.check("w", Value(640)).asInt64();
        h = rf.check("h", Value(480)).asInt64();
        period = rf.check("period", Value(0.004)).asFloat64();
        shape_w = rf.check("shape_w", Value(300)).asInt64();
        shape_h = rf.check("shape_h", Value(300)).asInt64();
        perc = rf.check("perc", Value(0.2)).asFloat64();

        // module name
        setName((rf.check("name", Value("/object_position")).asString()).c_str());

        state[0]=0; state[1]=0; state[2]=0; state[3]=1;

        background_image = cv::imread("/usr/local/src/affine_2d_tracking/desk_background.jpg", cv::IMREAD_COLOR);
        shape = cv::Mat::zeros(shape_h,shape_w, CV_8UC1);
        // shape_image = cv::imread("/usr/local/src/affine_2d_tracking/white_star.png", 0);
        shape_image = cv::imread("/usr/local/src/affine_2d_tracking/white_star.png", 0);

        cv::Rect shape_rect = cv::boundingRect(shape_image); 

        cut = shape_image(shape_rect);
        cv::resize(cut, cut, cv::Size(perc*w,perc*w)); 

        cv::resize(shape_image, shape_image, cv::Size(perc*w,perc*w)); 

        shape_image.copyTo(init_shape_image);
        cv::cvtColor(shape_image, color_shape_image, cv::COLOR_GRAY2RGB);

        cv::resize(background_image, resized_background, cv::Size(w,h)); 

        // square = cv::Rect(0,0, shape_w-4, shape_h-4);
        // cv::rectangle(shape, square, cv::Scalar(255),4,8,0);
        // cv::rectangle(shape, square, cv::Scalar(0),-(square.width-4),8,0);

        initial_position.x = resized_background.cols/2;
        initial_position.y = resized_background.rows/2;

        mask = cv::Rect(initial_position.x - shape_image.cols/2, initial_position.y - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        black_background = cv::Mat::zeros(resized_background.rows, resized_background.cols, CV_8U); 
        // // cut.copyTo(black_background(mask)); 

        // std::vector<std::vector<cv::Point> > contours;
        // std::vector<cv::Vec4i> hierarchy;
        // findContours( black_background, contours, hierarchy,cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

        // resized_background.copyTo(current_background); 

        // // iterate through all the top-level contours,
        // // draw each connected component with its own random color
        // int idx = 0;
        // for( ; idx >= 0; idx = hierarchy[idx][0] )
        // {
        //     cv::Scalar color(255, 255, 255);
        //     drawContours( current_background, contours, idx, color, cv::FILLED, 8, hierarchy );
        // }

        // createAffines(translation, initial_position, angle, pscale, nscale);

        // shift_right = updateTrMat(0, 1);
        // shift_left = updateTrMat(0, -1);

        // cv::namedWindow("test", cv::WINDOW_NORMAL);
        // cv::resizeWindow("test", cv::Size(w,h));
        // cv::moveWindow("test", 0, 0);

        // imshow("test", local_background);
        // cv::waitKey(0);

        std::array<double, 4> wayPoint1; 

        wayPoint1[0]=500;
        wayPoint1[1]=500;
        wayPoint1[2]=0;
        wayPoint1[3]=1;

        std::array<double, 4> wayPoint2; 
        
        wayPoint1[0]=100;
        wayPoint1[1]=100;
        wayPoint1[2]=10;
        wayPoint1[3]=1.5;

        return true;
    }

    bool updateModule(){


        // if (count<11 && first_time){
        //     yarp::os::Time::delay(2);
        //     if (count == 10)
        //         first_time = false; 
        //     if (count%2)
        //         sum_tx += 2;
        //     else
        //         sum_tx -= 2; 
        // }
        // else{
            // int random = rand() % 4;

            // if(random==0)
            //     sum_tx += 1;
            // else if (random == 1)
            //     sum_ty += 1; 
            // else if (random == 2)
            //     sum_rot += 1; 
            // else if (random == 3)
            //     sum_sc += 0.5;
            // else if (random == 4)
            //     sum_tx -= 1; 
            // else if (random == 5)
            //     sum_ty -= 1; 
            // else if (random == 6)
            //     sum_rot -= 1;
            // else if (random == 7)
            //     sum_sc -= 0.5; 

            if (!change_direction){
                sum_tx += 1;
                // sum_ty += 1; 
                // sum_rot += 0.5; 
                // tot_sc = tot_sc * 1.01;
    
            }
            else{
                sum_tx -= 1;
                // sum_ty -= 1; 
                // sum_rot -= 1;
                // tot_sc = tot_sc*0.99;
            }
        // }


        new_position.x = initial_position.x + sum_tx; 
        new_position.y = initial_position.y + sum_ty; 

        // resized_background.copyTo(local_background); 
        // shape.copyTo(local_background(new_mask)); 

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

        yInfo()<<sum_tx << sum_ty << sum_rot << sum_sc; 
    
        yInfo()<<w - shape_image.cols*2<<shape_image.cols*2;

        if(sum_tx >= w/2-shape_image.cols/2 && !change_direction){ // (sum_sc) > 25
            change_direction = true;
            // sum_sc = 0;
        }
        else if ( sum_tx <= -(w/2-shape_image.cols/2) && change_direction){ // (sum_sc) < -25
            change_direction = false;
            // sum_sc = 0;
        }

        imshow("test", current_background);
        cv::waitKey(1);

        yarp::os::Time::delay(1); 

        if (count<10)
            imwrite("/data/star_filled/translation_x/image-0000"+std::to_string(count)+".jpg", current_background);
        else if (count<100)
            imwrite("/data/star_filled/translation_x/image-000"+std::to_string(count)+".jpg", current_background);
        else if (count<1000)
            imwrite("/data/star_filled/translation_x/image-00"+std::to_string(count)+".jpg", current_background);
        else if (count<10000)
            imwrite("/data/star_filled/translation_x/image-0"+std::to_string(count)+".jpg", current_background);
        else
            imwrite("/data/star_filled/translation_x/image-"+std::to_string(count)+".jpg", current_background);

        count++;
        // yInfo()<<count<<new_position.x<<count_pass_by_origin; 

        if (new_position.x == w/2){
            if (count_pass_by_origin==1)
                return false; 
            count_pass_by_origin++; 
            yInfo()<<"hey"; 
        }

        // if (sum_sc == 0){
        //     if (count_pass_by_origin==1)
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