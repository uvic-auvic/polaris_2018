#include <ros/ros.h>
#include "navigation/nav.h"
#include "peripherals/motor.h"
#include "peripherals/motors.h"

class thrust_controller
{
public:
    thrust_controller(std::string node_name);
    void generate_thrust_val(const navigation::nav::ConstPtr &msg);
private:
    ros::NodeHandle nh;
    ros::ServiceClient motor_forward;
    ros::ServiceClient motor_reverse;
    ros::ServiceClient motor_set_all;
    ros::ServiceClient motor_stop;
    ros::ServiceClient motors_stop;
    const float EMatrix[5][5];
};

thrust_controller::thrust_controller(std::string node_name) :
    nh(ros::NodeHandle("~")),
    motor_forward(nh.serviceClient<peripherals::motor>(node_name + "/setMotorForward")),
    motor_reverse(nh.serviceClient<peripherals::motor>(node_name + "/setMotorReverse")),
    motor_set_all(nh.serviceClient<peripherals::motor>(node_name + "/setAllMotors")),
    motor_stop(nh.serviceClient<peripherals::motor>(node_name + "/stopMotors")),
    motors_stop(nh.serviceClient<peripherals::motor>(node_name + "/stopAllMotors")),
    EMatrix{
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0}
    } { }

void thrust_controller::generate_thrust_val(const navigation::nav::ConstPtr &msg)
{
/*    ROS_INFO("X: %.2f Y: %.2f Z: %.2f Speed: %d\n"
    , msg->direction.x, msg->direction.y, msg->direction.z, msg->speed);
*/}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrustmap");
    ros::NodeHandle nh("~");
    thrust_controller tc("motor_controller");
    ros::Subscriber joy = nh.subscribe<navigation::nav>
        ("/nav/navigation", 1, &thrust_controller::generate_thrust_val, &tc);
    ros::spin();
    return 0;
}
