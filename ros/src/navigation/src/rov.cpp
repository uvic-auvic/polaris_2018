#include <ros/ros.h>
#include "navigation/rov.h"

class gamepad_reciever {
public:
    gamepad_reciever() {}
    void recieve(const navigation::rov::ConstPtr& msg) {}
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rov");
    ros::NodeHandle nh("~");
    
    gamepad_reciever controls;
    ros::Subscriber sub = nh.subscribe<navigation::rov>("/rov/controls", 100, &gamepad_reciever::recieve, &controls);
    
    ros::spin();
    return 0;

}