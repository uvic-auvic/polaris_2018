#include <stdio.h>
#include <ros/ros.h>
#include "ai/start.h"

using startreq = ai::start::Request;
using startres = ai::start::Response;
using rosserv = ros::ServiceServer;

class autonomous_manager{
public:
    autonomous_manager() 
        :   nh(ros::NodeHandle("~")),
            rss(nh.advertiseService("start_ai", &autonomous_manager::start_autonomous_mode, this))
    {
        nh.getParam("can_start", can_start);
        nh.getParam("poll_delay", poll_delay);
        nh.getParam("start_delay", start_delay);
    }

    void start() {
        if (!can_start) {
            ROS_INFO("Waiting for User to let us start autonomous");
            ros::Duration delay(poll_delay);
            while (ros::ok() && !can_start) {
                delay.sleep();
                ros::spinOnce();
            }
        }

        ROS_INFO("Waiting for %d seconds before startinng...", start_delay);
        ros::Duration delay(start_delay);
        delay.sleep();
        
    }

    bool start_autonomous_mode(startreq& req, startres& res) {
        can_start = true;
        start_delay = req.delay_start;
        return true;
    }

private:
    ros::NodeHandle nh;
    rosserv rss;
    bool can_start;
    int poll_delay;
    int start_delay;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ai_master");
    ros::NodeHandle nh("~");
    autonomous_manager am;
    am.start();
    // Start doing AI things
    ROS_INFO("Starting Autonomous Mode");
    return 0;
}
