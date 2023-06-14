#include <yarp/os/all.h>
using namespace yarp::os;

#include "eros.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <random>

class objectPos: public yarp::os::RFModule
{
private:
    int w, h, shape_w, shape_h, motion_type;
    double period{0.1};
    std::string background_filename, shape_filename, output_filepath;
    bool background_dynamic{false}, sinusoid{false}; 

    cv::Point2d initial_position, new_position;

    cv::Mat shape_image, shift_right, shift_left, moved_shape, color_shape_image, warped_background;
    double sum_tx{0}, sum_ty{0};
    double sum_rot{0.0}, sum_sc{0.0}, tot_sc{1.0};
    bool change_direction{false};
    int count{0}, count_pass_by_origin{0};
    cv::Mat background_image, moving_background, resized_background, shape, local_background, current_background, cut, black_background, local_black_background, init_shape_image, warped_shifted_background;
    cv::Rect2d mask, new_mask, background_rect; 
    bool first_time{true}; 
    double perc{0.2};
    int counter_angle{0}, counter_sine{0}; 
    std::vector<double> random_x, random_y, angles_circle, time_sinusoid, x_sinusoidal, y_sinusoidal, theta_sinusoidal, sc_sinusoidal;

    std::ofstream gt_file;

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
        output_filepath = rf.check("output-path", Value("/data/moving_background/star/combined_motions")).asString(); 
        motion_type = rf.check("motion", Value(5)).asInt32(); // 1-> tx, 2-> ty, 3-> rot, 4-> scale, 5-> combined
        background_dynamic = rf.check("back_dynamic", Value(true)).asBool(); 
        sinusoid = rf.check("sin", Value(false)).asBool(); 

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

        // background_image = 0; ///uncomment to have black background
        cv::resize(background_image, resized_background, cv::Size(w,h)); 
        cv::resize(background_image, moving_background, cv::Size(w+200,h+200)); 

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
        waypoints.push_back({-200.0, -250.0, 20.0, 1.2, 0.0});
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

        gt_file.open("../gt_combined_motions.txt");
            if (!gt_file.is_open()) {
                yError() << "Could not open output file";
                return false;
        }

        if (background_dynamic){
            angles_circle = createArray(0, 360, 0.1); 
            int radius = 100;
            for( int i = 0; i < angles_circle.size(); i ++ ){
                random_x.push_back(radius*cos(angles_circle[i]*M_PI/180));
                random_y.push_back(radius*sin(angles_circle[i]*M_PI/180));
            }
        }

        time_sinusoid = createArray(0, 1, 0.0002); 
        double frequency = 1;
        for( int i = 0; i < time_sinusoid.size(); i ++ ){
            x_sinusoidal.push_back(((w-shape_image.cols)/2)*sin(2*M_PI*frequency*time_sinusoid[i]));            
            y_sinusoidal.push_back(((h-shape_image.rows)/2)*sin(2*M_PI*frequency*time_sinusoid[i]));
            theta_sinusoidal.push_back(90*sin(2*M_PI*frequency*time_sinusoid[i]));
            sc_sinusoidal.push_back(0.5+sin(2*M_PI*frequency*time_sinusoid[i]));
        }

        // yInfo()<<x_sinusoidal;
        // yInfo()<<" "; 
        // yInfo()<<y_sinusoidal; 
        // yInfo()<<" "; 
        // yInfo()<<theta_sinusoidal; 
        // yInfo()<<" "; 
        // yInfo()<<sc_sinusoidal; 

        return true;
    }

    double fRand(double fMin, double fMax)
    {
        double f = (double)rand() / RAND_MAX;
        return fMin + f * (fMax - fMin);
    }

    std::vector<double> createArray( int min, int max, double step )
    {
        std::vector<double> array;
        for( double i = min; i <= max; i += step )
        {
            array.push_back( i );
        }
        return array;
    }

    bool updateModule(){

        if(motion_type!=5){

        }

        black_background.copyTo(local_black_background); 

        if (counter_sine<time_sinusoid.size()){
            counter_sine++; 
        }
        else{
            counter_sine=0; 
        }

        if (sinusoid){

            if (motion_type == 1){
                new_position.x = initial_position.x + x_sinusoidal[counter_sine]; 
                new_position.y = h/2;
            }
            else if (motion_type == 2){
                new_position.y = initial_position.y + y_sinusoidal[counter_sine]; 
                new_position.x = w/2;
            }
            else if (motion_type == 3 || motion_type == 4){
                new_position.x = w/2;
                new_position.y = h/2;
            }
            else{
                sum_tx = interpolated[count].x;
                sum_ty = interpolated[count].y;
                sum_rot = interpolated[count].d;
                tot_sc = interpolated[count].s;

                new_position.x = initial_position.x + sum_tx; 
                new_position.y = initial_position.y + sum_ty; 
            }
        }
        else{
            
            if (motion_type!=5){
                if (!change_direction){
                    if (motion_type==1)
                        sum_tx += 1;
                    if (motion_type==2)
                        sum_ty += 1;
                    if (motion_type==3)
                        sum_rot += 0.3; 
                    if (motion_type==4)
                        tot_sc = tot_sc*1.001;
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
        }

        cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2d(init_shape_image.cols/2, init_shape_image.rows/2), sum_rot, tot_sc);
        cv::Mat warped_shape_image;
        warpAffine(init_shape_image, warped_shape_image, rot_mat, warped_shape_image.size());

        new_mask = cv::Rect(new_position.x - warped_shape_image.cols/2, new_position.y - warped_shape_image.rows/2, warped_shape_image.cols, warped_shape_image.rows); 

        warped_shape_image.copyTo(local_black_background(new_mask)); 

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours( local_black_background, contours, hierarchy,cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

        if (background_dynamic){
            if (counter_angle<angles_circle.size()){
                counter_angle++; 
            }
            else{
                counter_angle=0; 
            }
            // yInfo()<<random_x[counter_angle]<<random_y[counter_angle];

            // int random_x = rand() % 3 + (-1);
            // int random_y = rand() % 3 + (-1);
            // double random_rot = fRand(-0.2, 0.2);
            double random_rot = 0; 
            // double random_sc = fRand(0.98, 1.02);
            double random_sc = 1; 

            // yInfo()<<"ramdom val="<<random_x<<random_y<<random_rot<<random_sc; 

            cv::Mat random_rot_mat = cv::getRotationMatrix2D(cv::Point2d(moving_background.cols/2, moving_background.rows/2), random_rot, random_sc);
            random_rot_mat.at<double>(0,2) += (random_x[counter_angle]);
            random_rot_mat.at<double>(1,2) += (random_y[counter_angle]);
            warpAffine(moving_background, warped_background, random_rot_mat, warped_background.size());

            background_rect = cv::Rect(warped_background.cols/2 -w/2, warped_background.rows/2-h/2, w, h);
            warped_background(background_rect).copyTo(current_background);
        }
        else{
            resized_background.copyTo(current_background); 
        }

        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        int idx = 0;
        for( ; idx >= 0; idx = hierarchy[idx][0] )
        {
            cv::Scalar color(255, 255, 255);
            drawContours( current_background, contours, idx, color, cv::FILLED, 8, hierarchy );
        }

        if(((sum_tx >= w/2-shape_image.cols/2) || (sum_ty >= h/2-shape_image.rows/2) || sum_rot > 91 || tot_sc>1.5)&& !change_direction){ // (sum_sc) > 25
            change_direction = true;
            yInfo() << "direction changed";
            sum_sc = 0;
        }
        else if ((sum_tx <= -(w/2-shape_image.cols/2)|| (sum_ty <= -(h/2-shape_image.rows/2)) || sum_rot<-91 || tot_sc<0.5) && change_direction){ // (sum_sc) < -25
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

        gt_file<<sum_tx<<" "<<sum_ty<<" "<<tot_sc<<" "<<sum_rot<<std::endl;

        // yInfo()<<count<<new_position.x<<count_pass_by_origin; 

        // CODE TO STOP WHEN FIRST LOOP ENDED

        if (motion_type != 5){
            if (motion_type == 1){
                    if (new_position.x == w/2){
                        if (count_pass_by_origin==1)
                            return false; 
                        count_pass_by_origin++; 
                        yInfo()<<"hey"<<new_position.x; 
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
            if (count > interpolated.size() - 1)
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
        gt_file.close(); 
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