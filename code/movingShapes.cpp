#include <yarp/os/all.h>
using namespace yarp::os;

#include "eros.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

class objectPos: public yarp::os::RFModule
{
private:
    int w, h, shape_w, shape_h, motion_type;
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

    typedef struct affine_state {
        double x;
        double y;
        double d;
        double s;
        double score;
        friend std::ostream& operator<<(std::ostream& stream, const affine_state& as) {
            stream << std::fixed << std::setprecision(2) << "[" << as.x << " " << as.y << " " << as.d << " " << as.s << "]";
            return stream;
        }

        affine_state operator+(affine_state rhs) const {
            return {x + rhs.x, y + rhs.y, d + rhs.d, s + rhs.s, score};
        }

        affine_state operator-(affine_state rhs) const {
            return {x - rhs.x, y - rhs.y, d - rhs.d, s - rhs.s, score};
        }

        affine_state operator*(double t) const {
            return {x * t, y * t, d * t, s * t, score};
        }

    } affine_state;

    std::vector<affine_state> interpolated;

    //returns the linear interpolation of the affine state of size (k-1).n+1, where
    //k is the waypoints.size()
    std::vector<affine_state> interpolate_states(const std::vector<affine_state> &waypoints, int n)
    {
        size_t k_in = waypoints.size();
        size_t k_out = (k_in-1)*n;
        std::vector<affine_state> output(k_out);
        for(int i = 0; i < k_out; i++) 
        {
            int i_in = i / n;
            double j = (i % n)/(double)n;
            output[i] = waypoints[i_in] + (waypoints[i_in+1] - waypoints[i_in])*j;
        }
        output.push_back(waypoints.back());

        return output;
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
        shape_filename = rf.check("background-file", Value("/usr/local/src/affine2dtracking/shapes/star.png")).asString(); 
        output_filepath = rf.check("output-path", Value("/data/star_filled/combined_motions")).asString(); 
        motion_type = rf.check("motion", Value(5)).asInt32();  // 1-> tx, 2-> ty, 3-> rot, 4-> scale, 5-> combined

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
        if (motion_type==3 || motion_type == 4)
            cv::copyMakeBorder( init_shape_image, init_shape_image, 100, 100, 100, 100, cv::BORDER_CONSTANT, 0 );

        if (motion_type == 5)
            cv::copyMakeBorder( init_shape_image, init_shape_image, 50, 50, 50, 50, cv::BORDER_CONSTANT, 0 );

        cv::cvtColor(shape_image, color_shape_image, cv::COLOR_GRAY2RGB);

        // background_image = 0; ///uncomment to have black background
        cv::resize(background_image, resized_background, cv::Size(w,h)); 

        initial_position.x = resized_background.cols/2;
        initial_position.y = resized_background.rows/2;

        mask = cv::Rect(initial_position.x - shape_image.cols/2, initial_position.y - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        black_background = cv::Mat::zeros(resized_background.rows, resized_background.cols, CV_8U); 

        //create waypoints
        std::vector<affine_state> waypoints;
        waypoints.push_back({0.0, 0.0, 0.0, 1.0, 0.0});
        waypoints.push_back({100.0, 200.0, 45.0, 1.3, 0.0});
        waypoints.push_back({320.0, 30.0, -22.0, 0.9, 0.0});
        waypoints.push_back({600.0, 100.0, -22.0, 0.8, 0.0});
        waypoints.push_back({100.0, -200.0, -10.0, 1, 0.0});
        waypoints.push_back({-200.0, -300.0, 20.0, 1.2, 0.0});
        waypoints.push_back({-400.0, 50.0, 10.0, 0.95, 0.0});
        waypoints.push_back({-550.0, 200.0, 5.0, 0.9, 0.0});
        waypoints.push_back({-300.0, 150.0, 2.0, 1.1, 0.0});
        waypoints.push_back({0.0, 0.0, 0.0, 1.0, 0.0});

        interpolated = interpolate_states(waypoints, 250);

        std::cout << "OUPUT WAYPOINTS" << std::endl;
        std::cout << "---------------" << std::endl;
        for(auto i : interpolated)
            std::cout << i << std::endl;
        std::cout << std::endl;

        return true;
    }

    bool updateModule(){

        if(motion_type!=5){
            if (!change_direction){
                if (motion_type==1)
                    sum_tx += 1;
                if (motion_type==2)
                    sum_ty += 1;
                if (motion_type==3)
                    sum_rot += 0.3; 
                if (motion_type==4)
                    tot_sc = tot_sc*1.001;
                    // sum_sc += 0.05;
            }
            else{
                if (motion_type==1)
                    sum_tx -= 1;
                if (motion_type==2)
                    sum_ty -= 1;
                if (motion_type==3) 
                    sum_rot -= 0.3;
                if (motion_type==4)
                    tot_sc = tot_sc*0.999;
                    // sum_sc -= 0.05; 
            }
        }
        else{
            sum_tx = interpolated[count].x;
            sum_ty = interpolated[count].y;
            sum_rot = interpolated[count].d;
            tot_sc = interpolated[count].s;
        }

        new_position.x = initial_position.x + sum_tx; 
        new_position.y = initial_position.y + sum_ty; 

        black_background.copyTo(local_black_background); 

        cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2d(init_shape_image.cols/2, init_shape_image.rows/2), sum_rot, tot_sc);
        cv::Mat warped_shape_image;
        warpAffine(init_shape_image, warped_shape_image, rot_mat, warped_shape_image.size());
        // cv::resize(init_shape_image, warped_shape_image, cv::Size(warped_shape_image.cols+sum_sc, warped_shape_image.rows+sum_sc), 0, 0, cv::INTER_LINEAR);

        new_mask = cv::Rect(new_position.x - warped_shape_image.cols/2, new_position.y - warped_shape_image.rows/2, warped_shape_image.cols, warped_shape_image.rows); 

        warped_shape_image.copyTo(local_black_background(new_mask)); 

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

        if(((sum_tx >= w/2-shape_image.cols/2) || (sum_ty >= h/2-shape_image.rows/2) || sum_rot > 91 || sum_sc > 6 || tot_sc>1.5)&& !change_direction){ // (sum_sc) > 25
            change_direction = true;
            yInfo() << "direction changed";
            sum_sc = 0;
        }
        else if ((sum_tx <= -(w/2-shape_image.cols/2)|| (sum_ty <= -(h/2-shape_image.rows/2)) || sum_rot<-91 || sum_sc < -6 || tot_sc<0.5) && change_direction){ // (sum_sc) < -25
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

        // CODE TO STOP WHEN FIRST LOOP ENDED

        if (motion_type != 5){
            if (motion_type == 1){
                    std::cout<<sum_tx<<" "; 
                    if (new_position.x == w/2){
                        if (count_pass_by_origin==1)
                            return false; 
                        count_pass_by_origin++; 
                        yInfo()<<"hey"; 
                    }
                }
        
                if (motion_type == 2){
                    std::cout<<sum_ty<<" "; 
                    if (new_position.y == h/2){
                        if (count_pass_by_origin==1)
                            return false; 
                        count_pass_by_origin++; 
                        yInfo()<<"hey"; 
                    }
                }

                if(motion_type == 3){
                    std::cout<<sum_rot<<" "; 
                    if (sum_rot < 0.01 && sum_rot > -0.01){
                        if (count_pass_by_origin==1)
                            return false; 
                        count_pass_by_origin++; 
                        yInfo()<<"hey"; 
                    }
                }

                if (motion_type == 4){
                    // if (sum_sc < -5.95){
                    //     return false; 
                    // }

                    std::cout<<tot_sc<<" "; 
                    if (tot_sc < 1.001 && tot_sc > 1){
                        if (count_pass_by_origin==1)
                            return false; 
                        count_pass_by_origin++; 
                        yInfo()<<"hey"; 
                    }
                }
        }
        else{
            if (count > interpolated.size() -1)
                return false; 
        }

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