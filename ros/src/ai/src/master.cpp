#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <exception>
#include "ai/start.h"
#include "navigation/nav_request.h"
#include "navigation/depth_info.h"
#include "navigation/control_en.h"
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

enum states
{
    dive,
    dead_reckon_gate,
    dead_reckon_dice,
    gate_detect,
    dice_detect,
    stop,
    search,
    rise
};

class autonomous_manager{
public:
    autonomous_manager() 
        :   nh(ros::NodeHandle("~")),
            rss(nh.advertiseService("start_ai", &autonomous_manager::start_autonomous_mode, this)),
            nav_req_pub(nh.advertise<navigation::nav_request>("/nav/navigation", 1)),
	    control_en(nh.serviceClient<navigation::control_en>("/control_system/ControlSysEnable")),
            depth_delta(0.0),
            forwards_delta(0.0),
            sideways_delta(0.0),
            yaw_rate_delta(0.0),
            dice_en(false),
            scanner_en(false),
            max_dice_hit(false),
            min_dice_hit(false),
            dice_detected(false),
	    gate_detected(false),
	    gate_passed(false),
            fsm_state(dive),
            depth_ok(false)
    {
        nh.getParam("can_start", can_start);
        nh.getParam("poll_delay", poll_delay);
        nh.getParam("start_delay", start_delay);
        nh.getParam("depth_delta_max", depth_delta_max);
        nh.getParam("forwards_delta_max", forwards_delta_max);
        nh.getParam("sideways_delta_max", sideways_delta_max);
        nh.getParam("yaw_delta_max", yaw_delta_max);
        nh.getParam("depth_submerge", depth_submerge);
        nh.getParam("depth_rise", depth_rise);

        std::string forward_loc, reverse_loc, submerge_loc, rise_loc, rotate_cw_loc, rotate_ccw_loc, right_loc, left_loc;
        std::string forward_strt_loc, reverse_strt_loc, stop_loc;
        nh.getParam("forward_location", forward_loc);
        nh.getParam("reverse_location", reverse_loc);
        nh.getParam("submerge_location", submerge_loc);
        nh.getParam("rise_location", rise_loc);
        nh.getParam("rotate_cw_location", rotate_cw_loc);
        nh.getParam("rotate_ccw_location", rotate_ccw_loc);
        nh.getParam("right_location", right_loc);
        nh.getParam("left_location", left_loc);
        nh.getParam("forward_start_location", forward_strt_loc);
        nh.getParam("reverse_start_location", reverse_strt_loc);
        nh.getParam("stop_location", stop_loc);
        nh.getParam("depth_ok_thresh", depth_ok_thresh);
        nh.getParam("dead_reckon_gate_count", dead_reckon_gate_count);
        nh.getParam("dice_detect_count", dice_detect_count);
        nh.getParam("gate_detect_count", gate_detect_count);
        nh.getParam("search_count", search_count);

        if(!parse_json(forwards, forward_loc))
        {
            throw InvalidJSONException(forward_loc);
        }

        if(!parse_json(forwards_start, forward_strt_loc))
        {
            throw InvalidJSONException(forward_strt_loc);
        }

        if(!parse_json(reverse, reverse_loc))
        {
            throw InvalidJSONException(reverse_loc);
        }

        if(!parse_json(reverse_start, reverse_strt_loc))
        {
            throw InvalidJSONException(reverse_strt_loc);
        }

        if(!parse_json(stop, stop_loc))
        {
            throw InvalidJSONException(stop_loc);
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

    void run_forward() {run_event(forwards, depth_submerge);}
    void run_forward_start() {run_event(forwards_start, depth_submerge);}
    void run_reverse() {run_event(reverse, depth_submerge);}
    void run_reverse_start() {run_event(reverse_start, depth_submerge);}
    void run_submerge() {run_event(submerge, depth_submerge);}
    void run_rise() {run_event(rise, depth_rise);}
    void run_rotate_cw() {run_event(rotate_cw, depth_submerge);}
    void run_rotate_ccw() {run_event(rotate_ccw, depth_submerge);}
    void run_left() {run_event(left, depth_submerge);}
    void run_right() {run_event(right, depth_submerge);}
    void run_stop() {run_event(stop, depth_submerge);}

    void receive_cam_offset(const vision::offset_position::ConstPtr &msg);
    void receive_dice_offsets(const vision::dice_offsets::ConstPtr &msg);
    void receive_depth_info(const navigation::depth_info::ConstPtr &msg);

    states fsm_state;
    bool depth_ok;
    bool dice_en;
    bool scanner_en;
    bool dice_detected;
    bool gate_detected;
    bool gate_passed;
    int dead_reckon_gate_count;
    int dice_detect_count;
    int gate_detect_count;
    int search_count;
private:

    void run_event(std::vector<nav_event_t> event_list, double depth);

    ros::NodeHandle nh;
    ros::Publisher nav_req_pub;
    ros::ServiceClient control_en;
    rosserv rss;
    bool can_start;
    int poll_delay;
    int start_delay;
    navigation::nav_request last_nav_req;
    bool max_dice_hit;
    bool min_dice_hit;
    double depth_submerge;
    double depth_rise;
    double depth_ok_thresh;

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
    std::vector<nav_event_t> forwards_start;
    std::vector<nav_event_t> reverse;
    std::vector<nav_event_t> reverse_start;
    std::vector<nav_event_t> submerge;
    std::vector<nav_event_t> rise;
    std::vector<nav_event_t> rotate_cw;
    std::vector<nav_event_t> rotate_ccw;
    std::vector<nav_event_t> right;
    std::vector<nav_event_t> left;
    std::vector<nav_event_t> stop;

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
    if(scanner_en)
    {
	if(msg->relative_offset_x == 127)
	{
	    gate_passed = true;
	    return;
	}

	gate_detected = true;
	
	navigation::control_en srv;
	srv.request.vel_x_enable = true;
	srv.request.vel_y_enable = true;
	srv.request.vel_z_enable = true;
	srv.request.roll_enable = true;
	srv.request.pitch_enable = true;
	srv.request.yaw_enable = true;
	if(yaw_rate_delta * msg->relative_offset_x <= 0)
	{
	    srv.request.yaw_enable = false;
	}
	control_en.call(srv);

	yaw_rate_delta = ((double)msg->relative_offset_x) * yaw_delta_max / 100.0;

	navigation::nav_request nav_req = last_nav_req;
	nav_req.depth += depth_delta;
	nav_req.forwards_velocity += forwards_delta;
	nav_req.sideways_velocity += sideways_delta;
	nav_req.yaw_rate += yaw_rate_delta;
	
	nav_req_pub.publish(nav_req);

	srv.request.yaw_enable = true;
	control_en.call(srv);
    }
}

void autonomous_manager::receive_dice_offsets(const vision::dice_offsets::ConstPtr &msg)
{
    if(dice_en)
    {
        dice_detected = true;
	navigation::control_en srv;
	srv.request.vel_x_enable = true;
	srv.request.vel_y_enable = true;
	srv.request.vel_z_enable = true;
	srv.request.roll_enable = true;
	srv.request.pitch_enable = true;
	srv.request.yaw_enable = true;
	if(yaw_rate_delta * msg->max_dice_offset.x_offset <= 0)
	{
	    srv.request.yaw_enable = false;
	}
	if(depth_delta * msg->max_dice_offset.y_offset <= 0)
	{
	    srv.request.vel_z_enable = false;
	}
	control_en.call(srv);

        yaw_rate_delta = ((double)msg->max_dice_offset.x_offset) * yaw_delta_max / 100.0;
        depth_delta = ((double)msg->max_dice_offset.y_offset) * depth_delta_max / 100.0;

	navigation::nav_request nav_req = last_nav_req;
	nav_req.depth += depth_delta;
	nav_req.forwards_velocity += forwards_delta;
	nav_req.sideways_velocity += sideways_delta;
	nav_req.yaw_rate += yaw_rate_delta;
	
	nav_req_pub.publish(nav_req);

	srv.request.yaw_enable = true;
	srv.request.vel_z_enable = true;
	control_en.call(srv);
    }
}

void autonomous_manager::receive_depth_info(const navigation::depth_info::ConstPtr &msg)
{
    double signed_error = msg->desired_depth - msg->current_depth;
    double error = (signed_error < 0) ? -signed_error : signed_error;
    if(error < 0.07)
    {
        depth_ok = true;
    }
    else
    {
        depth_ok = false;
    }
}

void autonomous_manager::run_event(std::vector<nav_event_t> event_list, double depth)
{
    for(int i = 0; i < event_list.size() && ros::ok(); i++)
    {
        // Reset Deltas
        if(!scanner_en && !dice_en)
        {
            depth_delta = 0;
            forwards_delta = 0;
            sideways_delta = 0;
            yaw_rate_delta = 0;
        }
        
        // Get message and add deltas
        navigation::nav_request nav_req = event_list[i].nav_req;
        last_nav_req = nav_req;
	last_nav_req.depth = depth;
        nav_req.depth = depth + depth_delta;
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
	else
	{
	    ros::spin();
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
        nh.subscribe<vision::offset_position>("/video/scanned_" + front_cam_name, 1, &autonomous_manager::receive_cam_offset, &am);
    ros::Subscriber sub_dice_offsets = 
        nh.subscribe<vision::dice_offsets>("/video/dice_" + front_cam_name, 1, &autonomous_manager::receive_dice_offsets, &am);
    ros::Subscriber sub_depth = 
        nh.subscribe<navigation::depth_info>("/nav/depth_control_info", 1, &autonomous_manager::receive_depth_info, &am);
    
    am.start();

    if(!ros::ok())
    {
	return 0;
    }

    // Start doing AI things
    ROS_ERROR("Starting Autonomous Mode");
    
    // Calibrate pressure sensor
    peripherals::avg_data srv;
    srv.request.acq_rate = 30;
    srv.request.acq_count = 100;
    if(!nav_calib.call(srv))
    {
	ROS_ERROR("Failed to calibrate the system.");
	//return 1;
    }
    
    int state_count = 0;
    while(ros::ok())
    {
        switch(am.fsm_state)
        {
        case(dive):
            am.run_submerge();
            am.fsm_state = stop;
            break;
        case(dead_reckon_gate):
            am.run_forward();
            if(--state_count <= 0)
            {
                am.fsm_state = gate_detect;
                state_count = am.gate_detect_count;
                am.scanner_en = true;
            }
            break;
        case(gate_detect):
            am.scanner_en = true;
            am.run_forward();
            if(am.gate_passed)
            {
                am.fsm_state = dice_detect;
                state_count = am.dice_detect_count;
                am.scanner_en = false;
                am.run_forward();
                am.dice_en = true;
		am.scanner_en = true;
            }
	    am.gate_detected = false;
            break;
        case(dice_detect):
            am.dice_en = true;
            am.run_forward();
	    if(am.dice_detected)
	    {
		am.scanner_en = false;
		state_count = am.dice_detect_count;
	    }
	    else
	    {
		am.scanner_en = true;
	    }
            if(!am.gate_detected && !am.dice_detected && --state_count <= 0)
            {
                am.fsm_state = search;
                state_count = am.search_count;
                am.dice_en = true;
                am.scanner_en = false;
            }
            am.dice_detected = false;
	    am.gate_detected = false;
            break;
        case(stop):
            am.run_stop();
            if(am.depth_ok)
            {
                am.fsm_state = dead_reckon_gate;
                state_count = am.dead_reckon_gate_count;
                am.run_forward_start();
            }
            break;
        case(search):
            am.run_rotate_cw();
            if(am.dice_detected || --state_count <= 0)
            {
                am.run_forward_start();
                am.fsm_state = dice_detect;
                state_count = am.dice_detect_count;
                am.dice_en = true;
                am.scanner_en = false;
            }
            break;
        case(rise):
            am.run_rise();
            break;
        }

        ros::spinOnce();
    }
    return 0;
}
