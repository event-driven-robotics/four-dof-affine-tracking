#pragma once

#include <yarp/os/all.h>

using namespace yarp::os;

class affineTransforms
{
public:

    enum cam_param_name{w,h,cx,cy,fx,fy};

    typedef struct affine_struct
    {
        cv::Mat A;
        cv::Mat warped_img;
        double score;
        cv::Mat rmp;
        cv::Mat rmsp;
        int axis{0};
        double delta{0.0};

    } affine_bundle;

    std::array<affine_bundle, 9> affine_info;
    std::vector<cv::Mat> affines_vector;
    std::vector<double> scores_vector;
    std::array<double, 4> state;

    cv::Rect roi_around_shape, square;
    cv::Mat initial_template, roi_template, roi_template_64f, roi_resized, mexican_template, mexican_template_64f;
    cv::Mat eros_filtered, eros_tracked, eros_tracked_64f, eros_resized;
    cv::Mat rot_scaled_tr_template;
    double translation, angle, pscale, nscale, scaling;
    cv::Point initial_position, new_position;
    cv::Point2d new_center; 
    int blur{11};

    cv::Size proc_size{cv::Size(100, 100)};
    cv::Rect proc_roi; 
    cv::Mat prmx, prmy, nrmx, nrmy;


    std::array<double, 6> cam;

public:

    void init(double trans, double rot, double ps, double ns, double sc){
        
        // proc_roi = cv::Rect(cv::Point(0, 0), proc_size);

        for(auto &affine : affine_info) {
            affine.A = cv::Mat::zeros(2,3, CV_64F);
            // affine.warped_img = cv::Mat::zeros(proc_size, CV_64F);
        }

        prmx = cv::Mat::zeros(proc_size, CV_32F);
        prmy = cv::Mat::zeros(proc_size, CV_32F);
        nrmx = cv::Mat::zeros(proc_size, CV_32F);
        nrmy = cv::Mat::zeros(proc_size, CV_32F);

        this->translation = trans;
        this->angle = rot;
        this->pscale = ps;
        this->nscale = ns;
        this->scaling = sc;
    }

    void initState(){
        state[0]=0; state[1]=0; state[2]=0; state[3]=1;
    }

    void createAffines(double translation, cv::Point2f center, double angle, double pscale, double nscale){

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

    void updateAffines(){

        new_center = cv::Point2d(roi_around_shape.width/2, roi_around_shape.height/2);

        affine_info[4].A = cv::getRotationMatrix2D(new_center, angle, 1);
        affine_info[5].A = cv::getRotationMatrix2D(new_center, -angle, 1);

        affine_info[6].A = cv::getRotationMatrix2D(new_center, 0, pscale);
        affine_info[7].A = cv::getRotationMatrix2D(new_center, 0, nscale);
    }

    void setROI(int buffer = 20){

        static cv::Rect full_roi = cv::Rect(cv::Point(0, 0), rot_scaled_tr_template.size());

        roi_around_shape = cv::boundingRect(rot_scaled_tr_template);

        if(roi_around_shape.width == 0) {
            roi_around_shape = full_roi;
            return;
        }
        roi_around_shape.x -= buffer;
        roi_around_shape.y -= buffer;
        roi_around_shape.width += buffer * 2;
        roi_around_shape.height += buffer * 2;

        roi_around_shape = roi_around_shape & full_roi;

        roi_template = rot_scaled_tr_template(roi_around_shape);
        roi_template.convertTo(roi_template_64f, CV_64F);  // is 6 type CV_64FC1

        // cv::resize(roi_template_64f, roi_resized, proc_roi.size(), 0, 0, cv::INTER_CUBIC);

    }

    cv::Mat updateTrMat(double translation_x, double translation_y){

        // the state is an affine
        cv::Mat trMat = cv::Mat::zeros(2,3, CV_64F);

        trMat.at<double>(0,0) = 1; trMat.at<double>(0,1) = 0; trMat.at<double>(0,2) = translation_x;
        trMat.at<double>(1,0) = 0; trMat.at<double>(1,1) = 1; trMat.at<double>(1,2) = translation_y;

        return trMat;
    }

    void createStaticTemplate(const cv::Size &res, double scaling_percentage){

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);
        double square_width = res.height*(scaling_percentage/100);
        double square_height = res.height*(scaling_percentage/100);
        square = cv::Rect2d((res.width-square_width)/2,(res.height-square_height)/2, square_width, square_height);
        cv::rectangle(initial_template, square, cv::Scalar(255),1,8,0);

        initial_position.x = square.x + square.width/2;
        initial_position.y = square.y + square.height/2;

    }

    void loadTemplate(const cv::Size &res, std::string filename){

        cv::Mat shape_image = cv::imread(filename, 0);

        cv::resize(shape_image, shape_image, cv::Size(0.33*shape_image.cols, 0.33*shape_image.rows), 0, 0,  cv::INTER_LINEAR); 

        static cv::Mat shape_blur;
        cv::GaussianBlur(shape_image, shape_blur, cv::Size(3,3),0,0);

        // cv::Mat sobelxy;
        // cv::Sobel(shape_blur, sobelxy, CV_64F, 1, 1, 5);

        // static cv::Mat edges;
        // cv::Canny(shape_blur, edges, 100, 200, 3, false);

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);

        initial_position.x = res.width/2;
        initial_position.y = res.height/2;

        cv::Rect mask = cv::Rect(res.width/2 - shape_image.cols/2, res.height/2 - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        shape_image.copyTo(initial_template(mask)); 

    }

    void squareTemplate(const cv::Size &res){
        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);
        square = cv::Rect(306,263, 87, 93);
        cv::rectangle(initial_template, square, cv::Scalar(255),1,8,0);

        initial_position.x = square.x + square.width/2;
        initial_position.y = square.y + square.height/2;
    }

    // void createDynamicTemplate(){

    //     cv::Mat rotMatfunc = getRotationMatrix2D(initial_position, state[2], state[3]);
    //     rotMatfunc.at<double>(0, 2) += (state[0] - initial_position.x);
    //     rotMatfunc.at<double>(1, 2) += (state[1] - initial_position.y);
    //     cv::warpAffine(initial_template, rot_scaled_tr_template, rotMatfunc, rot_scaled_tr_template.size());
    //     new_position = cv::Point2d(initial_position.x+state[0],initial_position.y+state[1]);

    // }

    void createDynamicTemplate(){

        cv::Mat rot_scaled_template;

        cv::Mat rotMatfunc = getRotationMatrix2D(initial_position, state[2], state[3]);
        cv::warpAffine(initial_template, rot_scaled_template, rotMatfunc, rot_scaled_template.size());
        cv::Mat trMat =  updateTrMat(state[0], state[1]);
        cv::warpAffine(rot_scaled_template, rot_scaled_tr_template, trMat, rot_scaled_tr_template.size());
        new_position = cv::Point2d(initial_position.x+state[0],initial_position.y+state[1]);

    }

    void make_template() {
        
        static cv::Mat canny_img, f, pos_hat, neg_hat;
        static cv::Size pblur(blur, blur);
        static cv::Size nblur(2*blur-1, 2*blur-1);
        static double minval, maxval;

        // cv::GaussianBlur(roi_resized, pos_hat, pblur, 0);
        // cv::GaussianBlur(roi_resized, neg_hat, nblur, 0);
        cv::GaussianBlur(roi_template_64f, pos_hat, pblur, 0);
        cv::GaussianBlur(roi_template_64f, neg_hat, nblur, 0);
        mexican_template = pos_hat - neg_hat;

        cv::minMaxLoc(mexican_template, &minval, &maxval);
        double scale_factor = 1.0 / (2 * std::max(fabs(minval), fabs(maxval)));
        mexican_template *= scale_factor;
    }

    void createWarpings(){

        mexican_template.convertTo(mexican_template_64f, CV_64F);  // is 6 type CV_64FC1

        for (int affine = 0; affine < affine_info.size(); affine++) {
            cv::warpAffine(mexican_template_64f, affine_info[affine].warped_img, affine_info[affine].A, mexican_template_64f.size());
        }
    }

    void createMapWarpings(){

        mexican_template.convertTo(mexican_template_64f, CV_64F);  // is 6 type CV_64FC1

        for (int affine = 0; affine < affine_info.size()-1; affine++) {
            cv::remap(mexican_template_64f, affine_info[affine].warped_img, affine_info[affine].rmp, affine_info[affine].rmsp, cv::INTER_LINEAR);
            cv::imshow("affine remap" + std::to_string(affine), affine_info[affine].warped_img); 
        }

        affine_info[8].warped_img = mexican_template_64f; 
    
    }

    void setEROS(const cv::Mat &eros)
    {
        // cv::GaussianBlur(eros, eros_filtered, cv::Size(5, 5), 0);
        // cv::Mat eros_blurred1; 
        // cv::medianBlur(eros, eros_blurred1, 3);
        cv::GaussianBlur(eros, eros_filtered, cv::Size(3, 3), 0);
        eros_filtered(roi_around_shape).copyTo(eros_tracked);
        eros_tracked.convertTo(eros_tracked_64f, CV_64F, 0.003921569);
        // cv::resize(eros_tracked_64f, eros_resized, proc_roi.size(), 0, 0, cv::INTER_CUBIC);
    }

    double similarity_score(const cv::Mat &observation, const cv::Mat &expectation) {
        static cv::Mat muld;
        muld = expectation.mul(observation);
        return cv::sum(cv::sum(muld))[0];
    }

    void performComparisons(){

        for (int affine = 0; affine < affine_info.size(); affine++) {
            affine_info[affine].score = similarity_score(eros_tracked_64f, affine_info[affine].warped_img);
            // affine_info[affine].score = similarity_score(eros_resized, affine_info[affine].warped_img);
            scores_vector.push_back(affine_info[affine].score);
//            cv::imshow("affine"+std::to_string(affine), affine_info[affine].warped_img);
        }
    }

    void updateState(){

        int best_score_index = max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        // double best_score = *max_element(scores_vector.begin(), scores_vector.end());
        // yInfo() << scores_vector;
        // yInfo() << "highest score =" << best_score_index << best_score;
        scores_vector.clear();

        if (best_score_index == 0)
            state[0] += translation;
        else if (best_score_index == 1)
            state[0] -= translation;
        else if (best_score_index == 2)
            state[1] += translation;
        else if (best_score_index == 3)
            state[1] -= translation;
        else if (best_score_index == 4)
            state[2] += angle;
        else if (best_score_index == 5)
            state[2] -= angle;
        else if (best_score_index == 6)
            state[3] = state[3]*pscale;
        else if (best_score_index == 7)
            state[3] = state[3]*nscale;
    }

    void create_map_x(double dp, int affine_n){
        //x shift by dp
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                //positive
                prmx.at<float>(y, x) = x-dp;
                prmy.at<float>(y, x) = y;
                //negative
                nrmx.at<float>(y, x) = x+dp;
                nrmy.at<float>(y, x) = y;
            }
        }
        if (affine_n == 0){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = dp; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = -dp;
        }
        affine_info[affine_n].axis = affine_n;
    }

    void create_map_y(double dp, int affine_n){
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                //positive
                prmx.at<float>(y, x) = x;
                prmy.at<float>(y, x) = y-dp;
                //negative
                nrmx.at<float>(y, x) = x;
                nrmy.at<float>(y, x) = y+dp;
            }
        }

        if (affine_n == 2){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = dp; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = -dp;
        }
        affine_info[affine_n].axis = affine_n; 
    }

    void create_map_scale(double sc, int affine_n){
        double cy = proc_size.height * 0.5;
        double cx = proc_size.width  * 0.5;
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                double dx = -(x-cx) * sc;
                double dy = -(y-cy) * sc;
                //positive
                prmx.at<float>(y, x) = x - dx;
                prmy.at<float>(y, x) = y - dy;
                //negative
                nrmx.at<float>(y, x) = x + dx;
                nrmy.at<float>(y, x) = y + dy;
            }
        }
        if (affine_n == 6){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = sc; 
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = -sc;
        }
        affine_info[affine_n].axis = affine_n;
    }

    void create_map_rot(double angle, int affine_n){
        double cy = proc_size.height * 0.5;
        double cx = proc_size.width  * 0.5;
        double theta = atan2(angle, std::max(proc_size.width, proc_size.height)*0.5);
        for(int x = 0; x < proc_size.width; x++) {
            for(int y = 0; y < proc_size.height; y++) {
                double dx = -(y - cy) * cam[fx] / cam[fy] * theta;
                double dy =  (x - cx) * cam[fy] / cam[fx] * theta;
                //positive
                prmx.at<float>(y, x) = x - dx;
                prmy.at<float>(y, x) = y - dy;
                //negative
                nrmx.at<float>(y, x) = x + dx;
                nrmy.at<float>(y, x) = y + dy;
            }
        }
        if (affine_n == 6){
            cv::convertMaps(prmx, prmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = theta;
        }
        else{
            cv::convertMaps(nrmx, nrmy, affine_info[affine_n].rmp, affine_info[affine_n].rmsp, CV_16SC2);
            affine_info[affine_n].delta = -theta;
        }
        
        affine_info[affine_n].axis = affine_n;
    }

    void create_maps(){
        create_map_x(translation, 0); 
        create_map_x(translation, 1);
        create_map_y(translation, 2);
        create_map_y(translation, 3);
        create_map_rot(angle, 4);
        create_map_rot(angle, 5);
        create_map_scale(scaling, 6);
        create_map_scale(scaling, 7);
    }


};
