
#include <yarp/os/all.h>
#include <opencv2/opencv.hpp>
#include <event-driven/core.h>
#include <event-driven/algs.h>
#include <vector>
#include <thread>

class aFineShapeDetector 
{

public:
    typedef struct affine_state {
        int x;
        int y;
        double degs;
        double scale;
        double score;
    } affine_state;

public:
    cv::Size im_size{{640, 480}};
    std::vector<cv::Mat> shapes;
    std::vector<affine_state> states;
    cv::Mat observation;

public:
    void init(cv::Size sz) 
    {
        im_size = sz;
        observation = cv::Mat(im_size, CV_32F);
    };

    // creates
    void setObservation(cv::Mat eros) {
        // check all image types here
        static cv::Mat g = cv::getGaussianKernel(im_size.height, -1, CV_32F) * 
                         cv::getGaussianKernel(im_size.width,  -1, CV_32F).t();

        cv::Mat eros32; 
        eros.convertTo(eros32, CV_32F, 1.0/255.0);
        //eros *= 1.0/255.0;
        cv::Mat ret = 1.0 - eros32;
        ret = ret.mul(g);
        //observation = ret;

        cv::normalize(ret, observation, 0.0, 1.0, cv::NORM_MINMAX);
    }

    // creates some template shapes
    //  p is the percentage of the image size to make the shape
    void makeShapes(double p) {
        cv::Rect roi((im_size.width - (im_size.height*p))*0.5,
                     im_size.height * 0.5 * (1.0 - p),
                     im_size.height * p,
                     im_size.height * p);

        cv::Mat square = cv::Mat::zeros(im_size, CV_8U);
        cv::rectangle(square, roi, cv::Scalar(255), cv::FILLED);
        shapes.push_back(square);
    }

    // instead of making them every-time, maybe we want to load a set. that makes
    // sense also if the shapes are used in other applications.
    void loadShapes() {
        shapes.push_back(cv::imread("../objects/star640.png"));
        cv::imshow("shape", shapes.back());
        cv::waitKey();
        shapes.push_back(cv::imread("../objects/square640.png"));
        cv::imshow("shape", shapes.back());
        cv::waitKey();
        cv::destroyWindow("shape");
    }



    bool affineCheck(cv::Mat &m, int axis, double value, affine_state &state)
    {
        static const float onedeg = 1.0 * M_PI / 180.0;
        static const float onescale = 1.01;
        static std::array<double, 3> scores;
        cv::Mat M, M_minus, M_plus;
        affine_state A_minus = state, A_plus = state;
        M = cv::getRotationMatrix2D(cv::Point(m.size().width/2, m.size().height/2), state.degs, state.scale);
        M.copyTo(M_minus); M.copyTo(M_plus);
        switch(axis) {
            case(0): //x axis
                M_minus.at<float>(0, 2) = -1;
                M_plus.at<float>(0, 2)  =  1;
                A_minus.x -= 1; A_plus.x += 1;
                break;
            case(1): //y axis
                M_minus.at<float>(1, 2) = -1;
                M_plus.at<float>(1, 2)  =  1;
                A_minus.y -= 1; A_plus.y += 1;
                break;
            case(2): // theta
                M_minus = cv::getRotationMatrix2D(cv::Point(m.size().width/2, m.size().height/2), state.degs - onedeg, state.scale);
                M_plus  = cv::getRotationMatrix2D(cv::Point(m.size().width/2, m.size().height/2), state.degs + onedeg, state.scale);
                A_minus.degs -= onedeg; A_plus.x += onedeg;
                break;
            case(3): // scale
                M_minus = cv::getRotationMatrix2D(cv::Point(m.size().width/2, m.size().height/2), state.degs, state.scale / onescale);
                M_plus  = cv::getRotationMatrix2D(cv::Point(m.size().width/2, m.size().height/2), state.degs, state.scale * onescale);
                A_minus.scale /= onescale; A_plus.x *= onescale;
                break;
            default:
                yError() << "that's not an axis buddy";
                return false;
        }
        
        static cv::Mat muld;
        cv::Mat warped;
        affine_state best_affine = state;
        double best_score = -1, score;

        //set the initial score to the unwarped value
        cv::warpAffine(m, warped, M, warped.size());
        muld = warped.mul(observation);
        best_score =  cv::sum(cv::sum(muld))[0];
        bool ret = false;

        cv::warpAffine(m, warped, M_minus, warped.size());
        muld = warped.mul(observation);
        score = cv::sum(cv::sum(muld))[0];
        if(score > best_score) {
            best_score = score;
            state = A_minus;
            ret = true;
        }

        cv::warpAffine(m, warped, M_plus, warped.size());
        muld = warped.mul(observation);
        score =  cv::sum(cv::sum(muld))[0];
        if(score > best_score) {
            best_score = score;
            state = A_plus;
            ret = true;
        }

        return ret;

    }

    void affineSearch(cv::Mat shape, affine_state& state) {
        // do a convolution in space to find best candidate position location
        cv::Mat heat_map;
        cv::filter2D(observation, heat_map, -1, shape);
        double minv, maxv;
        cv::Point maxp;

        cv::minMaxLoc(heat_map, &minv, &maxv, nullptr, &maxp);

        state.x = maxp.x; state.y = maxp.y;
        
        // do a step-by-step search in all parameters


    }

    //search all shapes 
    int bestAffine(affine_state& state) {
        return 0;
    }

    affine_state fullSearch(cv::Mat &obs, cv::Mat &shp)
    {
        cv::Point c = {shp.size().width/2, shp.size().height/2};
        cv::Mat rshp, heat, rshp_roi_32;
        affine_state global_max = {0, 0, 0.0, 1.0, -DBL_MAX};
        for (double r = -180; r < 180; r += 2) {
            for (double s = 0.5; s < 1.5; s += 0.1) {
                cv::Mat m = cv::getRotationMatrix2D(c, r, s);
                cv::warpAffine(shp, rshp, m, shp.size());
                cv::Rect roi = cv::boundingRect(rshp);
                double energy = cv::sum(cv::sum(rshp))[0];
                rshp(roi).convertTo(rshp_roi_32, CV_32F);
                rshp_roi_32 *= (1.0/energy);
                cv::filter2D(obs, heat, -1, rshp_roi_32);
                double maxv; cv::Point maxp;
                cv::minMaxLoc(heat, nullptr, &maxv, nullptr, &maxp);
                if(maxv > global_max.score)
                    global_max = {maxp.x - c.x, maxp.y - c.y, r, s, maxv};
            }
        }
        return global_max;
    }

};

class rundemo : public yarp::os::RFModule
{
private:

    aFineShapeDetector afsd;
    ev::window<ev::AE> input;
    std::thread worker;
    ev::EROS eros;
    cv::Size im_sz{{640, 480}};

public:
    

    bool configure(yarp::os::ResourceFinder& rf) override
    {
        setName(rf.check("name", yarp::os::Value("/shapedetector")).asString().c_str());
        im_sz = {rf.check("width", yarp::os::Value(640)).asInt32(),
                 rf.check("height", yarp::os::Value(480)).asInt32()};
        afsd.init({640, 480});
        //afsd.makeShapes(0.4);
        afsd.loadShapes();
        if(!input.open(getName("/AE:i"))) {
            yError() << "Port Open Error" << getName("/AE:i");
            return false;
        }
        yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");

        eros.init(im_sz.width, im_sz.height, 7, 0.3);

        worker = std::thread([this]{erosupdate();});

        return true;
    }   

    double getPeriod() override
    {
        return 0.1;
    }

    bool interruptModule() override
    {
        input.stop();
        worker.join();
        return true;
    }

    bool updateModule() override
    {
        cv::Mat obs, obs_s, obs_s32;
        eros.getSurface().copyTo(obs);
        cv::GaussianBlur(obs, obs, cv::Size(9, 9), -1);
        cv::resize(obs, obs_s, im_sz/8);
        cv::GaussianBlur(obs_s, obs_s, {3, 3}, -1);
        cv::normalize(obs_s, obs_s, 0, 255, cv::NORM_MINMAX);
        obs_s.convertTo(obs_s32, CV_32F, 1.0/255.0);
        obs_s32 = 1.0 - obs_s32;
        //cv::Mat g = cv::getGaussianKernel(obs_s.size().height, -1, CV_32F) * 
        //            cv::getGaussianKernel(obs_s.size().width,  -1, CV_32F).t();
        //obs_s32 = obs_s32.mul(g);
        //cv::normalize(obs_s32, obs_s32, 0.0, 1.0, cv::NORM_MINMAX);

        cv::imshow("Observation Small", obs_s32);
        //cv::imshow("Observation big", obs);
        if(cv::waitKey(1) == 'g') {
            cv::Mat little_shape, best_warp, obs_sRGB;
            cv::Mat &my_shape = afsd.shapes.front();
            cv::resize(my_shape, little_shape, obs_s.size());
            aFineShapeDetector::affine_state b =  afsd.fullSearch(obs_s32, little_shape);

            cv::cvtColor(obs_s, obs_sRGB, cv::COLOR_GRAY2BGR);
            cv::circle(obs_sRGB, {b.x+obs_s.cols/2, b.y+obs_s.rows/2}, 2, cv::Vec3b(0, 255, 0), cv::FILLED); 

            cv::Point c = {im_sz.width/2, im_sz.height/2};

            cv::Mat M = cv::getRotationMatrix2D(c, b.degs, b.scale);
            M.at<double>(0, 2) += b.x*8;
            M.at<double>(1, 2) += b.y*8;
            cv::warpAffine(my_shape, best_warp, M, my_shape.size());


            cv::imshow("Best", best_warp+obs);
            cv::imshow("found", obs_sRGB);
        }

        return true;
    }

    void erosupdate()
    {
        while (!isStopping()) 
        {
            input.readAll();
            for (auto& v : input)
                eros.update(v.x, v.y);
        }
    }

};


int main(int argc, char* argv[])
{
        /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    /* create the module */
    rundemo instance;
    return instance.runModule(rf);
}