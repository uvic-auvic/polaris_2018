#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <exception>
#include "ai/start.h"
#include "navigation/nav_request.h"
#include "peripherals/avg_data.h"
#include "vision/offset_position.h"
#include "vision/dice_offsets.h"

using startreq = ai::start::Request;
using startres = ai::start::Response;
using rosserv = ros::ServiceServer;
using ptree = boost::property_tree::ptree;

class InvalidJSONException : public std::runtime_error {
public:
    InvalidJSONException(const std::string &file) : std::runtime_error("Invalid JSON file detected for \"" + file + "\"") {}
};

typedef struct nav_event
{
    double time;
    navigation::nav_request nav_req;
} nav_event_t;

class autonomous_manager{
public:
    autonomous_manager() 
        :   nh(ros::NodeHandle("~")),
            rss(nh.advertiseService("start_ai", &autonomous_manager::start_autonomous_mode, this)),
            nav_req_pub(nh.advertise<navigation::nav_request>("/nav/navigation", 1)),
            depth_delta(0.0),
            forwards_delta(0.0),
            sideways_delta(0.0),
            yaw_rate_delta(0.0),
            max_dice_hit(false),
            min_dice_hit(false),
            dice_detected(false)
    {
        nh.getParam("can_start", can_start);
        nh.getParam("poll_delay", poll_delay);
        nh.getParam("start_delay", start_delay);
        nh.getParam("depth_delta_max", depth_delta_max);
        nh.getParam("forwards_delta_max", forwards_delta_max);
        nh.getParam("sideways_delta_max", sideways_delta_max);
        nh.getParam("yaw_delta_max", yaw_delta_max);

        std::string forward_loc, reverse_loc, submerge_loc, rise_loc, rotate_cw_loc, rotate_ccw_loc, right_loc, left_loc;
        nh.getParam("forward_location", forward_loc);
        nh.getParam("reverse_location", reverse_loc);
        nh.getParam("submerge_location", submerge_loc);
        nh.getParam("rise_location", rise_loc);
        nh.getParam("rotate_cw_location", rotate_cw_loc);
        nh.getParam("rotate_ccw_location", rotate_ccw_loc);
        nh.getParam("right_location", right_loc);
        nh.getParam("left_location", left_loc);

        if(!parse_json(forwards, forward_loc))
        {
            throw InvalidJSONException(forward_loc);
        }

        if(!parse_json(reverse, reverse_loc))
        {
            throw InvalidJSONException(reverse_loc);
        }

        if(!parse_json(submerge, submerge_loc))
        {
            throw InvalidJSONException(submerge_loc);
        }

        if(!parse_json(rise, rise_loc))
        {
            throw InvalidJSONException(rise_loc);
        }

        if(!parse_json(rotate_cw, rotate_cw_loc))
        {
            throw InvalidJSONException(rotate_cw_loc);
        }

        if(!parse_json(rotate_cw, rotate_ccw_loc))
        {
            throw InvalidJSONException(rotate_ccw_loc);
        }

        if(!parse_json(left, left_loc))
        {
            throw InvalidJSONException(left_loc);
        }

        if(!parse_json(right, right_loc))
        {
            throw InvalidJSONException(right_loc);
        }
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

    void run_forward() {run_event(forwards);}
    void run_reverse() {run_event(reverse);}
    void run_submerge() {run_event(submerge);}
    void run_rise() {run_event(rise);}
    void run_rotate_cw() {run_event(rotate_cw);}
    void run_rotate_ccw() {run_event(rotate_ccw);}
    void run_left() {run_event(left);}
    void run_right() {run_event(right);}

    void receive_cam_offset(const vision::offset_position::ConstPtr &msg);
    void receive_dice_offsets(const vision::dice_offsets::ConstPtr &msg);
private:

    void run_event(std::vector<nav_event_t> event_list);

    ros::NodeHandle nh;
    ros::Publisher nav_req_pub; 
    rosserv rss;
    bool can_start;
    int poll_delay;
    int start_delay;
    bool max_dice_hit;
    bool min_dice_hit;
    bool dice_detected;

    // Placeholders for deltas
    double depth_delta;
    double forwards_delta;
    double sideways_delta;
    double yaw_rate_delta;

    // Limits for the deltas
    double depth_delta_max;
    double forwards_delta_max;
    double sideways_delta_max;
    double yaw_delta_max;

    // Navigation Event Lists
    std::vector<nav_event_t> forwards;
    std::vector<nav_event_t> reverse;
    std::vector<nav_event_t> submerge;
    std::vector<nav_event_t> rise;
    std::vector<nav_event_t> rotate_cw;
    std::vector<nav_event_t> rotate_ccw;
    std::vector<nav_event_t> right;
    std::vector<nav_event_t> left;

    bool parse_json(std::vector<nav_event_t> &nav_events, std::string json_file_location)
    {
        ptree pt;
        boost::property_tree::read_json(json_file_location, pt);

        // 1st dimension: holds all instances of nested repeats. First element is main.
        // 2nd dimension: used for all requests in a row for repititions
        std::vector<std::vector<nav_event_t>> nav_event_ordering;

        // Used to determine how many times to repeat each list in the list
        std::vector<int> repeat_counter;

        nav_event_ordering.push_back(std::vector<nav_event_t>());
        repeat_counter.push_back(1);

        for(ptree::const_iterator it = pt.begin(); it != pt.end(); ++it)
        {
            // Get data from JSON
            nav_event_t nav_event;
            nav_event.nav_req.forwards_velocity = it->second.get<double>("forwards_velocity");
            nav_event.nav_req.sideways_velocity = it->second.get<double>("sideways_velocity");
            nav_event.nav_req.yaw_rate = it->second.get<double>("yaw_rate");
            nav_event.nav_req.depth = it->second.get<double>("depth");
            nav_event.time = it->second.get<double>("time_ms");

            if(it->first.compare("single") == 0)
            {
                if(nav_event_ordering.size() > 1)
                {
                    ROS_ERROR("Invalid json. \"single\" cannot be inside a \"repeat\".");
                    return false;
                }
                nav_event_ordering.front().push_back(nav_event);
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

                nav_event_ordering.push_back(std::vector<nav_event_t>());
                repeat_counter.push_back(count);

                nav_event_ordering.back().push_back(nav_event);
            }
            else if(it->first.compare("repeat") == 0)
            {
                if(nav_event_ordering.size() == 1)
                {
                    ROS_ERROR("Invalid json. \"repeat\" must be between a \"repeat_start\" and a \"repeat_end\".");
                    return false;
                }
                nav_event_ordering.back().push_back(nav_event);
            }
            else if(it->first.compare("repeat_end") == 0)
            {
                if(nav_event_ordering.size() == 1)
                {
                    ROS_ERROR("Invalid json. \"repeat_end\" must come after a \"repeat_start\".");
                    return false;
                }

                // Add this request to current loop
                nav_event_ordering.back().push_back(nav_event);

                // Pop off last loop of loop list
                std::vector<nav_event_t> nav_loop = nav_event_ordering.back();
                int repeat_count = repeat_counter.back();
                nav_event_ordering.pop_back();
                repeat_counter.pop_back();

                // Add loop for the required number of times to its outer loop
                for(int i = 0; i < repeat_count; i++)
                {
                    nav_event_ordering.back().insert(nav_event_ordering.back().end(), nav_loop.begin(), nav_loop.end());
                }
            }
        }

        if(nav_event_ordering.size() > 1)
        {
            ROS_ERROR("Invalid json. Unterminated loop. A \"repeat_start\" must terminate with a \"repeat_end\"");
            return false;
        }

        nav_events = nav_event_ordering.front();

        return true;
    }

};

void autonomous_manager::receive_cam_offset(const vision::offset_position::ConstPtr &msg)
{
    yaw_rate_delta = msg->relative_offset_x * yaw_delta_max / 100.0;
}

void autonomous_manager::receive_dice_offsets(const vision::dice_offsets::ConstPtr &msg)
{
    yaw_rate_delta = msg->max_dice_offset.x * yaw_delta_max / 100.0;
    depth_delta = -msg->max_dice_offset.y * depth_delta_max / 100.0;
    
    /*dice_detected = true;
    if(!max_dice_hit)
    {
        yaw_rate_delta = msg->max_dice_offset.x * yaw_delta_max / 100.0;
        depth_delta = -msg->max_dice_offset.y * depth_delta_max / 100.0;
    }
    else if(!min_dice_hit)
    {
        yaw_rate_delta = msg->min_dice_offset.x * yaw_delta_max / 100.0;
        depth_delta = -msg->min_dice_offset.y * depth_delta_max / 100.0;
    }*/
}

void autonomous_manager::run_event(std::vector<nav_event_t> event_list)
{
    for(int i = 0; i < event_list.size() && ros::ok(); i++)
    {
        // Get message and add deltas
        navigation::nav_request nav_req = event_list[i].nav_req;
        nav_req.depth += depth_delta;
        nav_req.forwards_velocity += forwards_delta;
        nav_req.sideways_velocity += sideways_delta;
        nav_req.yaw_rate += yaw_rate_delta;

        // Publish message
        nav_req_pub.publish(nav_req);
        
        if(event_list[i].time > 0)
        {
            ros::Duration d(event_list[i].time / 1000.0);
            ros::spinOnce();
            d.sleep();
        }
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ai_master");
    ros::NodeHandle nh("~");

    std::string front_cam_name;
    nh.getParam("front_cam_name", front_cam_name);

    autonomous_manager am;
    ros::Publisher nav_req_pub = nh.advertise<navigation::nav_request>("/nav/navigation", 1);
    ros::ServiceClient nav_calib = nh.serviceClient<peripherals::avg_data>("/nav/CalibrateSurfaceDepth");
    ros::Subscriber sub_front_cam_offsets = 
        nh.subscribe<vision::offset_position>("/vision/" + front_cam_name, 1, &autonomous_manager::receive_cam_offset, &am);
    ros::Subscriber sub_dice_offsets = 
        nh.subscribe<vision::offset_position>("/vision/dice_offsets", 1, &autonomous_manager::receive_dice_offsets, &am);
    
    am.start();

    // Start doing AI things
    ROS_ERROR("Starting Autonomous Mode");
    
    // Calibrate pressure sensor
    peripherals::avg_data srv;
    srv.request.acq_rate = 30;
    srv.request.acq_count = 100;
    if(!nav_calib.call(srv))
    {
	ROS_ERROR("Failed to calibrate the system.");
	return 1;
    }

    am.run_forward();
    
    ros::spin();

    return 0;
}
