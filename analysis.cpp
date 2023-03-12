#include <yarp/os/all.h>
using namespace yarp::os;

#include "eros.h"

class objectPos: public yarp::os::RFModule
{
private:
    int w, h;
    double period{0.1};
    int blur{21};

    ev::window<ev::AE> input_port;

    ev::EROS eros;
    int eros_k;
    double eros_d;
    cv::Mat eros_filtered, eros_detected, eros_detected_roi, eros_tracked, eros_tracked_roi, eros_tracked_32f, eros_tracked_64f;

    int init_filter_width, init_filter_height;
    double detection_thresh;
    cv::Rect detection_roi;
    cv::Rect roi_full;

    cv::Point object_position;

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

    void squareTemplate(){
        square = cv::Rect(306,263, 87, 93);
        cv::rectangle(initial_template, square, cv::Scalar(255),1,8,0);

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

        initial_position.x = 325;
        initial_position.y = 219;
    }

    double similarity_score(const cv::Mat &observation, const cv::Mat &expectation) {
        static cv::Mat muld;
        muld = expectation.mul(observation);
        return cv::sum(cv::sum(muld))[0];
    }

    void loadTemplate(const cv::Size &res, std::string filename){

        cv::Mat shape_image = cv::imread(filename, 0);

        cv::resize(shape_image, shape_image, cv::Size(0.5*shape_image.cols, 0.5*shape_image.rows), 0, 0,  cv::INTER_LINEAR); 

        initial_template = cv::Mat::zeros(res.height, res.width, CV_8UC1);

        initial_position.x = res.width/2 + shape_image.rows/2;
        initial_position.y = res.height/2 + shape_image.cols/2;

        cv::Rect mask = cv::Rect(res.width/2 - shape_image.cols/2, res.height/2 - shape_image.rows/2, shape_image.cols, shape_image.rows); 

        shape_image.copyTo(initial_template(mask)); 

    }

    void printMatrix(cv::Mat &matrix) const {
        for (int r=0; r < matrix.rows; r++){
            for (int c=0; c<matrix.cols; c++){
                std::cout<<matrix.at<double>(r,c)<<" ";
            }
            std::cout << std::endl;
        }
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

    void make_template(const cv::Mat &input, cv::Mat &output) {
        static cv::Mat  pos_hat, neg_hat;
        static cv::Size pblur(blur, blur);
        static cv::Size nblur(2*blur-1, 2*blur-1);
        static double minval, maxval;

        cv::GaussianBlur(input, pos_hat, pblur, 0);
        cv::GaussianBlur(input, neg_hat, nblur, 0);
        output = pos_hat - neg_hat;

        cv::minMaxLoc(output, &minval, &maxval);
        double scale_factor = 1.0 / (2 * std::max(fabs(minval), fabs(maxval)));
        output *= scale_factor;
    }

    cv::Mat createDynamicTemplate(std::array<double, 4> state){

        cv::Mat rot_scaled_template, rot_scaled_tr_template;

        cv::Mat rotMatfunc = getRotationMatrix2D(initial_position, state[2], state[3]);
        cv::warpAffine(initial_template, rot_scaled_template, rotMatfunc, rot_scaled_template.size());
        cv::Mat trMat =  updateTrMat(state[0], state[1]);
        cv::warpAffine(rot_scaled_template, rot_scaled_tr_template, trMat, rot_scaled_tr_template.size());
        new_position = cv::Point2d(initial_position.x+state[0],initial_position.y+state[1]);

        return rot_scaled_tr_template;
    }

    void createWarpings(){

        mexican_template.convertTo(mexican_template_64f, CV_64F);  // is 6 type CV_64FC1

        for (int affine = 0; affine < affine_info.size(); affine++) {
//                cv::warpAffine(roi_template_64f, affine_info[affine].warped_img, affine_info[affine].A, roi_template_64f.size());
            cv::warpAffine(mexican_template_64f, affine_info[affine].warped_img, affine_info[affine].A, mexican_template_64f.size());
        }
    }

    void setROI(cv::Mat full_template, int buffer = 20){

        roi_around_shape = cv::boundingRect(full_template);
        roi_around_shape.x -= buffer;
        roi_around_shape.y -= buffer;
        roi_around_shape.width += buffer * 2;
        roi_around_shape.height += buffer * 2;

        roi_template = full_template(roi_around_shape);
        roi_template.convertTo(roi_template_64f, CV_64F);  // is 6 type CV_64FC1
    }

    void updateAffines(){

        cv::Point2d new_center(roi_around_shape.width/2, roi_around_shape.height/2);

        affine_info[4].A = cv::getRotationMatrix2D(new_center, angle, 1);
        affine_info[5].A = cv::getRotationMatrix2D(new_center, -angle, 1);

        affine_info[6].A = cv::getRotationMatrix2D(new_center, 0, pscale);
        affine_info[7].A = cv::getRotationMatrix2D(new_center, 0, nscale);
    }

    void performComparisons(){
        eros_tracked.convertTo(eros_tracked_64f, CV_64F);  // is 6 type CV_64FC1

        for (int affine = 0; affine < affine_info.size(); affine++) {
            affine_info[affine].score = similarity_score(eros_tracked_64f, affine_info[affine].warped_img);
            scores_vector.push_back(affine_info[affine].score);
//            cv::imshow("affine"+std::to_string(affine), affine_info[affine].warped_img);
        }
    }

    void updateState(){

        int best_score_index = max_element(scores_vector.begin(), scores_vector.end()) - scores_vector.begin();
        //double best_score = *max_element(scores_vector.begin(), scores_vector.end());
        //yInfo() << scores_vector;
        //yInfo() << "highest score =" << best_score_index << best_score;
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

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // options and parameters
        w = rf.check("w", Value(640)).asInt64();
        h = rf.check("h", Value(480)).asInt64();
        eros_k = rf.check("eros_k", Value(17)).asInt32();
        eros_d = rf.check("eros_d", Value(0.3)).asFloat64();
        period = rf.check("period", Value(0.01)).asFloat64();

        // module name
        setName((rf.check("name", Value("/object_position")).asString()).c_str());

        eros.init(w, h, eros_k, eros_d);

        if (!input_port.open("/object_position/AE:i")){
            yError()<<"cannot open input port";
            return false;
        }

//        yarp::os::Network::connect("/file/leftdvs:o", "/object_position/AE:i", "fast_tcp");
        yarp::os::Network::connect("/atis3/AE:o", "/object_position/AE:i", "fast_tcp");

        state[0]=0; state[1]=0; state[2]=0; state[3]=1;

        // CREATE THE B&W TEMPLATE
        initial_template = cv::Mat::zeros(h, w, CV_8UC1);
        // starTemplate();
        squareTemplate();
        // loadTemplate(cv::Size(w,h), "/usr/local/src/affine_2d_tracking/star-removebg-preview.png");

        createAffines(translation, initial_position, angle, pscale, nscale);

        affine_thread = std::thread([this]{fixed_step_loop();});

        return true;
    }

    void fixed_step_loop() {

        while (!input_port.isStopping()) {

            // 1) create 1 dynamic template
//            yInfo()<<"State ="<<state[0]<<state[1]<<state[2]<<state[3];
            cv::Mat dynamic_template = createDynamicTemplate(state);

            // 2) find the roi (getboundingbox function opencv)
            setROI(dynamic_template);
            updateAffines();

            // 3) mexican hat
            make_template(roi_template_64f, mexican_template);

            // 4) create 9 templates (warped images)
            createWarpings();

            // 5) get EROS ROI
            ev::info my_info = input_port.readChunkT(0.004, true);
            for (auto &v : input_port)
                eros.update(v.x, v.y);

            cv::GaussianBlur(eros.getSurface(), eros_filtered, cv::Size(5, 5), 0);
            eros_filtered(roi_around_shape).copyTo(eros_tracked); // is 0 type CV_8UC1

            // 6) compare 9 templates vs eros
            performComparisons();

            // 7) update the state based on maximum store
            updateState();

            // 8) visualize
            cv::Mat norm_mexican;
            cv::normalize(mexican_template_64f, norm_mexican, 1, 0, cv::NORM_MINMAX);
            imshow("MEXICAN ROI", mexican_template_64f+0.5);
            imshow("TEMPLATE ROI", roi_template);
            imshow("EROS ROI", eros_tracked);
            cv::circle(eros_filtered, new_position, 2, 255, -1);
            cv::rectangle(eros_filtered, roi_around_shape, 255,1,8,0);
            imshow("EROS FULL", eros_filtered+dynamic_template);
//            imshow("TEMPLATE FULL", dynamic_template);
            cv::waitKey(0);

        }
    }

    bool updateModule(){
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