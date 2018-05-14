#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>

#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "monitor/GetSerialDevice.h"

using rosserv = ros::ServiceServer;

class power_board{
public:
    power_board(std::string port, int baud_rate = 9600, int timeout = 1000);
    ~power_board();
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::string write(std::string out, bool ignore_response = true, std::string eol = "\n");
};

power_board::power_board(std::string port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to power_board on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
}

power_board::~power_board() {
    connection->close();
}

std::string power_board::write(std::string out, bool ignore_response, std::string eol)
{
    connection->write(out + eol);
    ROS_INFO("%s", out.c_str());
    if (ignore_response) {
        return "";
    }
    return connection->readline(65536ul, eol);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "power_board");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv)) {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Power Board on fd %s\n", srv.response.device_fd.c_str());

    /* Wait for callbacks */
    ros::spin();

    return 0;
}

