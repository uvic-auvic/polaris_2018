#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>
#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "monitor/GetSerialDevice.h"

#define NUM_MOTORS (8)
#define NUM_CHAR_PER_MOTOR (2)

using MotorReq = peripherals::motor::Request;
using MotorRes = peripherals::motor::Response;
using MotorsReq = peripherals::motors::Request;
using MotorsRes = peripherals::motors::Response;
using rosserv = ros::ServiceServer;

class motor_controller {
public:
    motor_controller(const std::string & port, int baud_rate = 9600, int timeout = 1000);
    ~motor_controller();
    bool setMotorForward(MotorReq &, MotorRes &);
    bool setMotorReverse(MotorReq &, MotorRes &);
    bool setAllMotors(MotorsReq &req, MotorsRes &res);
    bool stopMotor(MotorReq &req, MotorRes &res);
    bool stopAllMotors(MotorReq &, MotorRes &);
    bool getRPM(MotorsReq &, MotorsRes &);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::string write(const std::string & out, bool ignore_response = true, std::string eol = "\n");
};

motor_controller::motor_controller(const std::string & port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to motor_controller on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
}

motor_controller::~motor_controller() {
    this->write("STP");
    connection->close();
}

std::string motor_controller::write(const std::string & out, bool ignore_response, std::string eol)
{
    connection->write(out + eol);
    //ROS_INFO("%s", out.c_str());
    if (ignore_response) {
        return "";
    }
    return connection->readline(65536ul, eol);
}

bool motor_controller::setMotorForward(MotorReq &req, MotorRes &res)
{
    std::string out = "M" + std::to_string(req.motor_number) + "F";
    out += std::to_string(req.motor_argument / 10) + std::to_string(req.motor_argument % 10);
    this->write(out);
    return true;
}

bool motor_controller::setMotorReverse(MotorReq &req, MotorRes &res)
{
    std::string out = "M" + std::to_string(req.motor_number) + "R";
    out += std::to_string(req.motor_argument / 10) + std::to_string(req.motor_argument % 10);
    this->write(out);
    return true;
}

bool motor_controller::setAllMotors(MotorsReq &req, MotorsRes &res)
{
    std::string out = "MSA";
    for (auto motor_power : req.arguments) {
        std::string dir = "F";
        if (motor_power < 0) {
            dir = "R";
            motor_power *= -1;
        }
        out += dir + std::to_string(motor_power / 10) + std::to_string(motor_power % 10);
    }
    this->write(out);
    return true;
}

bool motor_controller::stopMotor(MotorReq &req, MotorRes &res)
{
    std::string out = "SM" + std::to_string(req.motor_number);
    this->write(out);
    return true;
}

bool motor_controller::stopAllMotors(MotorReq &req, MotorRes &res)
{
    this->write("STP");
    return true;
}

bool motor_controller::getRPM(MotorsReq &req, MotorsRes &res)
{
    std::string rpm_string = this->write("RVA", false);
    ROS_INFO("Got: \"%s\"", rpm_string.c_str());
    if (rpm_string.size() != (NUM_CHAR_PER_MOTOR * NUM_MOTORS) + 2) { //assuming eol is \r\n
        return false;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        std::string ascii_str = rpm_string.substr(i*2, 2);
        res.results[i] = std::stoi(ascii_str);
    } 

    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_con");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv)) {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Motor Controller on fd %s\n", srv.response.device_fd.c_str());

    motor_controller m(srv.response.device_fd);

    /* Setup all the Different services/commands which we  can call. Each service does its own error handling */
    rosserv mf  = nh.advertiseService("setMotorForward", &motor_controller::setMotorForward, &m);
    rosserv mr  = nh.advertiseService("setMotorReverse", &motor_controller::setMotorReverse, &m);
    rosserv stp = nh.advertiseService("stopAllMotors", &motor_controller::stopAllMotors, &m);
    rosserv sm  = nh.advertiseService("stopMotors", &motor_controller::stopMotor, &m);
    rosserv rpm = nh.advertiseService("getRPM", &motor_controller::getRPM, &m);
    rosserv sam = nh.advertiseService("setAllMotors", &motor_controller::setAllMotors, &m);

    /* Wait for callbacks */
    ros::spin();

    return 0;
}

