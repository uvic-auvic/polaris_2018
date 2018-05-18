#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>

#include "monitor/GetSerialDevice.h"
#include "peripherals/powerboard.h"

using rosserv = ros::ServiceServer;
using powerboardInfo = peripherals::powerboard;

class power_board{
public:
    power_board(const std::string & port, int baud_rate = 9600, int timeout = 1000);
    ~power_board();
    void get_powerboard_data(powerboardInfo & msg);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::string write(const std::string & out, bool ignore_response = true, std::string eol = "\n");
};

power_board::power_board(const std::string & port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to power_board on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
}

power_board::~power_board() {
    connection->close();
}

std::string power_board::write(const std::string & out, bool ignore_response, std::string eol)
{
    connection->write(out + eol);
    ROS_INFO("%s", out.c_str());
    if (ignore_response) {
        return "";
    }
    return connection->readline(65536ul, eol);
}

void power_board::get_powerboard_data(powerboardInfo &msg) {
    std::string currents = this->write("CRA", false);
    std::string voltages = this->write("VTA", false);
    std::string temperature = this->write("TMP", false);
    std::string humidity = this->write("HUM", false);
    std::string water = this->write("WTR", false);
    std::string pressure_internal = this->write("PIN");
    std::string pressure_external = this->write("PEX");

    msg.current_all = 50;
    msg.current_battery_1 = 50;
    msg.current_battery_2 = 50;
    msg.current_motors = 50;
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

