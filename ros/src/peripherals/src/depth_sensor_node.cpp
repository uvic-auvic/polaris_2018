#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "peripherals/depth.h"

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
    ros::NodeHandle nh("~");

    // Get args from launch file
    int loop_rate, buffered_messages;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("buffered_messages", buffered_messages);
    
    // Declare publisher
    ros::Publisher pub = nh.advertise<peripherals::depth>("depth", buffered_messages);

    ros::Rate r(loop_rate);
    while(ros::ok())
    {
        peripherals::depth msg;

        msg.temperature = temperature_from_sensor();
        msg.depth = depth_from_sensor();
        
        pub.publish(msg);
        
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}