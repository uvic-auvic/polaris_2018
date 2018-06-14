#include <ros/ros.h>
#include <sstream>
#include <string>
#include "navigation/joystick.h"
#include "navigation/keyboard.h"
#include "navigation/nav.h"

class rov_mapper {
public:
    rov_mapper(); 
    void recieve_joystick(const navigation::joystick::ConstPtr& msg);
    void recieve_keyboard(const navigation::keyboard::ConstPtr& msg);
private:
    ros::NodeHandle nh;
    ros::Publisher nav_pub;
    bool W_pressed;
    bool A_pressed;
    bool S_pressed;
    bool D_pressed;
};

rov_mapper::rov_mapper() 
    :   nh(ros::NodeHandle("~")),
        nav_pub(nh.advertise<navigation::nav>("/nav/nav", 5)),
        W_pressed(false),
        A_pressed(false),
        S_pressed(false),
        D_pressed(false) {}

void rov_mapper::recieve_joystick(const navigation::joystick::ConstPtr& msg) {
    
    bool fire = msg->buttons[0];
    bool stop_pressed = msg->buttons[3];
    bool up_pressed = msg->buttons[1];
    bool down_pressed = msg->buttons[2];

    // Set default values
    navigation::nav nav_msg;
    nav_msg.direction.x = 0; // forwards or backwards;
    nav_msg.direction.y = 0;// left or right;
    nav_msg.direction.z = 0;// up or down;
    nav_msg.rotation.roll = 0;
    nav_msg.rotation.pitch = 0;
    nav_msg.rotation.yaw = 0;

    // check if we should stop everything
    // handy workaround to the keyboard-browser issue
    if (stop_pressed) {
        nav_pub.publish(nav_msg);
        return;
    }

    // Check if we should go up or not
    if (up_pressed) {
        nav_msg.direction.z = 10;
        nav_pub.publish(nav_msg);
        return;
    }
    
    if (down_pressed) {
        nav_msg.direction.z = -10;
        nav_pub.publish(nav_msg);
        return;
    }

    if (W_pressed) {
        nav_msg.direction.x = 10.0;
        nav_msg.direction.y = msg->axes[0];
        nav_msg.direction.z = msg->axes[1];
        nav_pub.publish(nav_msg);
        return;
    } 
    
    if (S_pressed) {
        nav_msg.direction.x = -10.0;
        nav_msg.direction.y = -msg->axes[0];
        nav_msg.direction.z = -msg->axes[1];
        nav_pub.publish(nav_msg);
        return;
    }

    if (A_pressed) {
        nav_msg.rotation.roll = 10;
        nav_pub.publish(nav_msg);
        return;
    }

    if (S_pressed) {
        nav_msg.rotation.roll = -10;
        nav_pub.publish(nav_msg);
        return;
    }

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
    
    rov_mapper controls;
    ros::Subscriber joy = nh.subscribe<navigation::joystick>("/nav/joystick", 1, &rov_mapper::recieve_joystick, &controls);
    ros::Subscriber key = nh.subscribe<navigation::keyboard>("/nav/keyboard", 1, &rov_mapper::recieve_keyboard, &controls);
    ros::spin();
    return 0;

}