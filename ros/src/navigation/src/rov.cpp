#include <sstream>
#include <string>
#include <ros/ros.h>
#include "navigation/joystick.h"
#include "navigation/keyboard.h"
#include "navigation/nav_request.h"

#define DEPTH_DELTA (0.02)
#define MAX_DEPTH (100)
#define MIN_DEPTH (0)

class rov_mapper {
public:
    rov_mapper(double max_speed_ms, double max_yaw_rate_degs); 
    void recieve_joystick(const navigation::joystick::ConstPtr &msg);
    void recieve_keyboard(const navigation::keyboard::ConstPtr &msg);
private:
    ros::NodeHandle nh;
    ros::Publisher nav_pub;
    bool W_pressed;
    bool A_pressed;
    bool S_pressed;
    bool D_pressed;
    double depth;
    const double max_speed_ms;
    const double max_yaw_rate_degs;
};

rov_mapper::rov_mapper(double max_speed_ms, double max_yaw_rate_degs) 
    :   nh(ros::NodeHandle("~")),
        nav_pub(nh.advertise<navigation::nav_request>("/nav/navigation", 5)),
        W_pressed(false),
        A_pressed(false),
        S_pressed(false),
        D_pressed(false),
        depth(0.0),
        max_speed_ms(max_speed_ms),
        max_yaw_rate_degs(max_yaw_rate_degs) {}

void rov_mapper::recieve_joystick(const navigation::joystick::ConstPtr& msg) {
    // Check buttons on joystick
    bool fire = msg->buttons[0];
    bool stop_pressed = msg->buttons[3];
    bool up_pressed = msg->buttons[1];
    bool down_pressed = msg->buttons[2];

    // Set default values
    navigation::nav_request nav_msg;
    nav_msg.depth = depth;
    nav_msg.yaw_rate = 0;
    nav_msg.forwards_velocity = 0;
    nav_msg.sideways_velocity = 0;

    // check if we should stop everything
    // handy workaround to the keyboard-browser issue
    if (stop_pressed) {
        nav_pub.publish(nav_msg);
        return;
    }

    // Check if we should go up or not
    if (up_pressed) {
        depth -= DEPTH_DELTA; // Depth is in the downwards direction
        if(depth < MIN_DEPTH) { 
            depth = MIN_DEPTH;
        }
        nav_msg.depth = depth;
        nav_pub.publish(nav_msg);
    }
    
    if (down_pressed) {
        depth += DEPTH_DELTA; // Depth is in the downwards direction
        if(depth > MAX_DEPTH) { 
            depth = MAX_DEPTH;
        }
        nav_msg.depth = depth;
        nav_pub.publish(nav_msg);
    }

    // Joystick axes go from [-100 100]. We want to go from [-max_speed max_speed]
    nav_msg.sideways_velocity = (max_speed_ms * (double)msg->axes[0]) / 100.0;
    nav_msg.forwards_velocity = -(max_speed_ms * (double)msg->axes[1]) / 100.0;
    nav_msg.yaw_rate = -(max_yaw_rate_degs * (double)msg->axes[2]) / 100.0;

    // set motor to stop
    nav_pub.publish(nav_msg);
}

void rov_mapper::recieve_keyboard(const navigation::keyboard::ConstPtr& msg) {
    W_pressed = msg->W_pressed;
    A_pressed = msg->A_pressed;
    S_pressed = msg->S_pressed;
    D_pressed = msg->D_pressed;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "rov");
    ros::NodeHandle nh("~");

    double max_speed_ms, max_yaw_rate_degs;
    nh.getParam("max_speed_ms", max_speed_ms);
    nh.getParam("max_yaw_rate_degs", max_yaw_rate_degs);
    
    rov_mapper controls(max_speed_ms, max_yaw_rate_degs);
    ros::Subscriber joy = nh.subscribe<navigation::joystick>("/nav/joystick", 1, &rov_mapper::recieve_joystick, &controls);
    ros::Subscriber key = nh.subscribe<navigation::keyboard>("/nav/keyboard", 1, &rov_mapper::recieve_keyboard, &controls);
    ros::spin();
    return 0;
}
