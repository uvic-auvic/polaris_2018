#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "ai/start.h"
#include "navigation/nav_request.h"
#include "peripherals/avg_data.h"

using startreq = ai::start::Request;
using startres = ai::start::Response;
using rosserv = ros::ServiceServer;
using ptree = boost::property_tree::ptree;

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

void parse_json(std::vector<navigation::nav_request> &nav_reqs, std::vector<double> &times, std::string json_file_location)
{
    ptree pt;
    boost::property_tree::read_json(json_file_location, pt);

    ROS_ERROR("Parsing data!");

    for(ptree::const_iterator it = pt.begin(); it != pt.end(); ++it)
    {
        // Get data from JSON
        navigation::nav_request nav_req;
        nav_req.forwards_velocity = it->second.get<double>("forwards_velocity");
        nav_req.sideways_velocity = it->second.get<double>("sideways_velocity");
        nav_req.yaw_rate = it->second.get<double>("yaw_rate");
        nav_req.depth = it->second.get<double>("depth");
        double time = it->second.get<double>("time_ms");

        // Append data to output
        nav_reqs.push_back(nav_req);
        times.push_back(time);
    }

    ROS_ERROR("Data parsed!");
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ai_master");
    ros::NodeHandle nh("~");
    autonomous_manager am;
    ros::Publisher nav_req_pub = nh.advertise<navigation::nav_request>("/nav/navigation", 1);
    ros::ServiceClient nav_calib = nh.serviceClient<peripherals::avg_data>("/nav/CalibrateSurfaceDepth");
    am.start();
    // Start doing AI things
    ROS_INFO("Starting Autonomous Mode");

    // Calibrate pressure sensor
    peripherals::avg_data srv;
    srv.request.acq_rate = 30;
    srv.request.acq_count = 100;
    if(!nav_calib.call(srv))
    {
	ROS_INFO("Failed to calibrate the system.");
	return 1;
    }

    std::vector<navigation::nav_request> nav_order;
    std::vector<double> timing;

    std::string json_location;
    nh.getParam("json_location", json_location);
    parse_json(nav_order, timing, json_location);

    for(int i = 0; i < nav_order.size(); i++)
    {
        ROS_ERROR("i = %d", i);
        nav_req_pub.publish(nav_order[i]);
        
        if(timing[i] > 0)
        {
            ros::Duration d(timing[i] / 1000.0);
            d.sleep();
        }
    }

    ros::spin();

    return 0;
}
