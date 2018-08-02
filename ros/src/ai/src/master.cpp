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

bool parse_json(std::vector<navigation::nav_request> &nav_reqs, std::vector<double> &times, std::string json_file_location)
{
    ptree pt;
    boost::property_tree::read_json(json_file_location, pt);

    // 1st dimension: holds all instances of nested repeats. First element is main.
    // 2nd dimension: used for all requests in a row for repititions
    std::vector<std::vector<navigation::nav_request>> nav_ordering;
    std::vector<std::vector<double>> time_ordering;

    // Used to determine how many times to repeat each list in the list
    std::vector<int> repeat_counter;

    nav_ordering.push_back(std::vector<navigation::nav_request>());
    time_ordering.push_back(std::vector<double>());
    repeat_counter.push_back(1);

    for(ptree::const_iterator it = pt.begin(); it != pt.end(); ++it)
    {
        // Get data from JSON
        navigation::nav_request nav_req;
        nav_req.forwards_velocity = it->second.get<double>("forwards_velocity");
        nav_req.sideways_velocity = it->second.get<double>("sideways_velocity");
        nav_req.yaw_rate = it->second.get<double>("yaw_rate");
        nav_req.depth = it->second.get<double>("depth");
        double time = it->second.get<double>("time_ms");

        if(it->first.compare("single") == 0)
        {
            if(nav_ordering.size() > 1)
            {
                ROS_ERROR("Invalid json. \"single\" cannot be inside a \"repeat\".");
                return false;
            }
            nav_ordering.front().push_back(nav_req);
            time_ordering.front().push_back(time);
        }
        else if(it->first.compare(0, 12, "repeat_start") == 0)
        {
            size_t dash_idx = it->first.find('-');
            std::string repeat_count = it->first.substr(dash_idx + 1, it->first.size() - (dash_idx + 1)); 
            if(repeat_count == "")
            {
                ROS_ERROR("Invalid json. \"repeat_start\" must be written in the following format: \"repeat_start-x\", where x is the number of repititions");
                return false;
            }
            int count = std::stoi(repeat_count);

            nav_ordering.push_back(std::vector<navigation::nav_request>());
            time_ordering.push_back(std::vector<double>());
            repeat_counter.push_back(count);

            nav_ordering.back().push_back(nav_req);
            time_ordering.back().push_back(time);
        }
        else if(it->first.compare("repeat") == 0)
        {
            if(nav_ordering.size() == 1)
            {
                ROS_ERROR("Invalid json. \"repeat\" must be between a \"repeat_start\" and a \"repeat_end\".");
                return false;
            }
            nav_ordering.back().push_back(nav_req);
            time_ordering.back().push_back(time);
        }
        else if(it->first.compare("repeat_end") == 0)
        {
            if(nav_ordering.size() == 1)
            {
                ROS_ERROR("Invalid json. \"repeat_end\" must come after a \"repeat_start\".");
                return false;
            }

            // Add this request to current loop
            nav_ordering.back().push_back(nav_req);
            time_ordering.back().push_back(time);

            // Pop off last loop of loop list
            std::vector<navigation::nav_request> nav_loop = nav_ordering.back();
            std::vector<double> time_loop = time_ordering.back();
            int repeat_count = repeat_counter.back();
            nav_ordering.pop_back();
            time_ordering.pop_back();
            repeat_counter.pop_back();

            // Add loop for the required number of times to its outer loop
            for(int i = 0; i < repeat_count; i++)
            {
                nav_ordering.back().insert(nav_ordering.back().end(), nav_loop.begin(), nav_loop.end());
                time_ordering.back().insert(time_ordering.back().end(), time_loop.begin(), time_loop.end());
            }
        }
    }

    if(nav_ordering.size() > 1)
    {
        ROS_ERROR("Invalid json. Unterminated loop. A \"repeat_start\" must terminate with a \"repeat_end\"");
        return false;
    }

    nav_reqs = nav_ordering.front();
    times = time_ordering.front();

    return true;
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
    if(!parse_json(nav_order, timing, json_location))
    {
        ROS_ERROR("Failed to parse json. Exiting.");
        return 1;
    }

    for(int i = 0; i < nav_order.size(); i++)
    {
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
