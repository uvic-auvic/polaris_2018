#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "depth_sensor/depth_msg.h"

using namespace std;

ros::Publisher pub;

int depth_from_sensor()
{
    /***** TODO: ******/
    /** IMPLEMENT ME **/
    return 5;
}

int temperature_from_sensor()
{
    /***** TODO: ******/
    /** IMPLEMENT ME **/
    return 7;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "depth_sensor_node");
    ros::NodeHandle nh;

    pub = nh.advertise<depth_sensor::depth_msg>("depth", 5);

    ros::Rate r(10); // 10 Hz
    while(ros::ok())
    {
        depth_sensor::depth_msg msg;

        msg.temperature = temperature_from_sensor();
        msg.depth = depth_from_sensor();
        
        pub.publish(msg);
        r.sleep();
    }

    ros::spin();
    return 0;
}