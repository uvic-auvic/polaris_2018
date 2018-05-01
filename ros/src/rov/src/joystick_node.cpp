/********************************************************************
 * @file /src/joystick_node.cpp
 * @brief The program that interfaces with the joystick controller
 * @date October 2017
 * @IMPORTANT INFO you will have to modify the msg, CMakeLists.txt, package.xml,
   launchfiles and joystick_node.cpp in respects to your catkin package
   ex: this was set up for my package "rov" in my catkin workspace, yours or 
   or the sumbmarine will also have a diffrent package name
/********************************************************************/

/********************************************************************
 * Includes
/********************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <unistd.h>
#include <iostream>
#include "rov/joystick_message.h"
#include "joystick.h"

/********************************************************************
 * Implementation [Main]
 * @Args     argc is the number of command-line arguments provided
 * @Args     argv is a pointer to the argument strings
/********************************************************************/
int main(int argc, char **argv){
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle nh("~");

    /* Launch Parameters*/
    std::string joy_fd, joy_pub_name, msg;
    int looping_rate, sleep_time, timeout_time;

    /* get Parameters */
     nh.getParam("joy_fd", joy_fd);
     nh.getParam("joy_pub_name", joy_pub_name);
     nh.getParam("looping_rate", looping_rate);
     nh.getParam("sleep_time", sleep_time);
     nh.getParam("timeout_time", timeout_time);

    /* Setup Node */
    ros::Publisher joy_pub = nh.advertise<rov::joystick_message>(joy_pub_name, 10);
    Joystick joystick(joy_fd);
    ros::Rate loop_rate(looping_rate);

    /* Sleep for 1s if no Joystick is found. Shutdown node after 60s */
    int timeout_counter = timeout_time;
    while(!joystick.isFound())
    {
        usleep(sleep_time);
         if (!timeout_counter) {
            return -1;
        }
    }

    ros::spinOnce();

    JoystickEvent prev_axis_x, prev_axis_y, prev_axis_v;
    bool forward_cmd, up_cmd, down_cmd = false;
    prev_axis_x.number = 0;
    prev_axis_y.number = 1;
    prev_axis_v.number = 2;
    
    /* Poll for Joystick Samples */
    while(ros::ok()) {
        JoystickEvent event;
        rov::joystick_message msgs;
        
        if (joystick.sample(&event)) {
            int eventNumber = (int) event.number;
            int eventValue  = (int) event.value;

            if (event.isButton()){ //button handling below, sets flags for respective commands mapped  to buttons

                if(event.number == 0 && event.value == 1){
                    forward_cmd = true;
                }else if(event.number == 0 && event.value == 0){
                    forward_cmd = false;
                }else if(event.number == 1 && event.value == 1){
                    up_cmd = true;
                }else if(event.number == 1 && event.value == 0){
                    up_cmd = false;
                }else if(event.number == 2 && event.value == 1){
                    down_cmd = true;
                }else if(event.number == 2 && event.value == 0){
                    down_cmd = false;
                } 
                
            }else if (event.isAxis()) { //sets the input velocity cap and if foward_cmd flag is on, sets the x,y,z 
                                        //values for direction input
                if(event.number == prev_axis_v.number){
                         prev_axis_v.value = event.value;
                }else if(forward_cmd == true){

                    if(event.number == prev_axis_x.number){
                         prev_axis_x.value = event.value;
                    }else if(event.number == prev_axis_y.number){
                         prev_axis_y.value = event.value;
                    } 
                }
            }
        }

            int x_value  = (int) prev_axis_x.value;
            int y_value = (int) prev_axis_y.value;
            int v_value = ((((int) prev_axis_v.value) - (-32767)) * 100)/ 65534; //formula for mapping old range to range of  0-100

            if(forward_cmd == true && up_cmd != true && down_cmd != true){ //forward command joystick input
                msgs.x = x_value;
                msgs.y = y_value;
                msgs.z = 1;

            }else if(up_cmd == true && down_cmd !=true && forward_cmd !=true){ //up only command   
                msgs.x = 0;
                msgs.y = 0;
                msgs.z = 1;

            }else if(up_cmd != true and down_cmd == true && forward_cmd !=true){ //down only command
                msgs.x = 0;
                msgs.y = 0;
                msgs.z = -1;
            }     

        msgs.v = v_value;
        joy_pub.publish(msgs);

        ros::spinOnce();
        loop_rate.sleep();
        
    }      

    ROS_INFO("END");
    return 0;
}
