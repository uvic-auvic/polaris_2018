#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>
#include <iostream>

#include "monitor/GetSerialDevice.h"
#include "peripherals/imu.h"

using rosserv = ros::ServiceServer;
using imu_msg = peripherals::imu;

class imu{
public:
    imu(const std::string & port, int baud_rate = 11520, int timeout = 3000);
    ~imu();
    void get_temperature();
private:
    void write(uint8_t command, int response_bytes = 0);
    std::unique_ptr<serial::Serial> connection = nullptr;
    uint8_t * response_buffer = nullptr;
};

imu::imu(const std::string & port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to imu on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
    response_buffer = new uint8_t[7];
}

imu::~imu() {
    connection->close();
    delete [] response_buffer;
}

void imu::write(uint8_t command, int response_bytes)
{
    connection->write(&command, 1);
    //connection->flushInput();
    
    // Put result into vector if need be
    if (response_bytes) {
        connection->read(response_buffer, (size_t) response_bytes);
    }
}

void imu::get_temperature() {
    write(0x07, 7);
    double temperature = (( (double)(((int) response_buffer[1] << 8) | (int) response_buffer[2]) * 5.0 / 65536) - 0.5) * 100.0;;
    ROS_INFO("Temp is : %f\n", temperature);
    ROS_INFO("Byte values are: ");
    for (int i= 0; i < 7; i ++) {
        ROS_INFO(" %d ", response_buffer[i]);
    }
}



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    /*
    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv)) {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }
    */

    //ROS_INFO("Using imu on fd %s\n", srv.response.device_fd.c_str());

    /* Wait for callbacks */
    //ros::spin();
    imu dev("/dev/ttyUSB0");
    ros::Rate r(4);
    while(ros::ok()) {
        dev.get_temperature();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

