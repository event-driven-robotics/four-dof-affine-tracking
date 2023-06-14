#pragma once

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Time.h>
// #include <yarp/rosmsg/std_msgs/Int64.h>
// #include <yarp/rosmsg/geometry_msgs/Pose.h>
#include <yarp/rosmsg/SharedData.h>

using yarp::os::Network;
using yarp::os::Node;
using yarp::os::Publisher;

namespace {
    YARP_LOG_COMPONENT(TALKER, "yarp.example.ros.talker")
    constexpr double loop_delay = 0.1;
}

class YarpToRos
{

private:

public:

    // yarp::os::Publisher<yarp::rosmsg::geometry_msgs::Pose> publisher;
    yarp::os::Publisher<yarp::rosmsg::SharedData> port;  // changed Port to Publisher

    yarp::os::Node* node = nullptr;

    // yarp::rosmsg::geometry_msgs::Pose data;

    yarp::rosmsg::SharedData d;

    void initPublisher(){
       // Network yarp;
        /* creates a node called /yarp/talker */
        node = new yarp::os::Node("/yarp/talker");

       if (!port.topic("foo2/sharedmessage"))              // replaced open() with topic()
       {
           yCError(TALKER) << "Failed to create publisher to /position";
       }

        /* subscribe to topic chatter */
        // if (!publisher.topic("/star_position")) {
        //     yCError(TALKER) << "Failed to create publisher to /catcher/event/object_position";
        // }
    }

    void publishTargetPos(cv::Size resolution, double u, double v, double theta, double sc){

        double u_ref = resolution.width/2;
        double v_ref = resolution.height/2;
        double theta_ref = 0;
        double sc_ref = 1; 

        // data.position.x = u-u_ref;
        // data.position.y = v-v_ref;
        // data.position.z = sc- sc_ref;
        // data.orientation.x = theta -theta_ref; 
        // yInfo()<<u-u_ref<<v-v_ref<< theta-theta_ref<<sc-sc_ref;
        d.content.push_back(u-u_ref);
        d.content.push_back(v-v_ref);
        d.content.push_back(theta-theta_ref);
        d.content.push_back(sc-sc_ref);
        // yInfo()<<d.content;
        // std::cout<<std::endl; 
        // std::cout<<std::endl; 

        port.write(d);

        d.content.clear();

        /* publish it to the topic */
        // publisher.write(data);
    }

    ~YarpToRos()
    {
        if (node)
        {
            delete node;
            node = nullptr;
        }
    }

};

