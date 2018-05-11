#include <ros/ros.h>
#include <iomanip>
#include <string>
#include <sstream>
#include <serial/serial.h>
#include <memory>

#include "peripherals/motor.h"
#include "peripherals/motors.h"

#define NUM_MOTORS (8)
#define NUM_CHAR_PER_MOTOR (2)

using MotorReq = peripherals::motor::Request;
using MotorRes = peripherals::motor::Response;
using MotorsReq = peripherals::motors::Request;
using MotorsRes = peripherals::motors::Response;

class motor_controller {
public:
    motor_controller(std::string, int, int);
    ~motor_controller();
    bool setMotorForward(MotorReq &, MotorRes &);
    bool setMotorReverse(MotorReq &, MotorRes &);
    bool setAllMotors(MotorsReq &req, MotorsRes &res);
    bool stopMotor(MotorReq &req, MotorRes &res);
    bool stopAllMotors(MotorReq &, MotorRes &);
    bool getRPMs(MotorsReq &, MotorsRes &);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::string write(std::string, bool, std::string);
};

motor_controller::motor_controller(std::string port, int baud_rate = 9600, int timeout = 1000) {
    ROS_INFO("Connecting to motor_controller on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
}

std::string motor_controller::write(std::string out, bool ignore_response = true, std::string eol = "\n")
{
    connection->write(out + eol);
    if (ignore_response) {
        return "";
    }
    return connection->readline(65536ul, eol);
}

motor_controller::~motor_controller() {
    std::string stop("STP");
    this->write(stop);
    connection->close();
}

bool motor_controller::setMotorForward(MotorReq &req, MotorRes &res)
{
    std::string out = "M" + std::to_string(req.motor_number) + "F";
    std::stringstream motor_param;
    motor_param << std::setw(2) << std::setfill('0') << req.motor_argument;
    out += motor_param.str();
    this->write(out);
    return true;
}

bool motor_controller::setMotorReverse(MotorReq &req, MotorRes &res)
{
    std::string out = "M" + std::to_string(req.motor_number) + "R";
    std::stringstream motor_param;
    motor_param << std::setw(2) << std::setfill('0') << req.motor_argument;
    out += motor_param.str();
    this->write(out);
    return true;
}

bool motor_controller::setAllMotors(MotorsReq &req, MotorsRes &res)
{
    std::string out = "MSA";
    std::stringstream motor_param;
    for (auto motor_power : req.arguments) {
        std::string dir = motor_power > 0 ? "F" : "R";
        motor_param << dir << std::setw(2) << std::setfill('0') << motor_power;
    }
    out += motor_param.str();
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
    std::string stop("STP");
    this->write(stop);
    return true;
}

bool motor_controller::getRPMs(MotorsReq &req, MotorsRes &res)
{
    std::string rpm_string = this->write("RVA", false);

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

    std::string fd;
    nh.getParam("motor_port", fd);

    /* Setup all the Different services/commands which we  can call. Each service does its own error handling */
    //ros::ServiceServer setMotorForwardService = nh.advertiseService("setMotorForward", setMotorForward);
    //ros::ServiceServer setMotorReverseService = nh.advertiseService("setMotorReverse", setMotorReverse);
    //ros::ServiceServer stopAllMotorsService   = nh.advertiseService("stopAllMotors", stopAllMotors);
    //ros::ServiceServer stopMotorService       = nh.advertiseService("stopMotors", stopMotor);
    //ros::ServiceServer getRPMService          = nh.advertiseService("getRPM", getRPM);
    //ros::ServiceServer setPWMService          = nh.advertiseService("setRPM", setPWM);
    //ros::ServiceServer calibrateMotorService  = nh.advertiseService("calibrateMotor", calibrateMotor);

    /* Wait for callbacks */
    ros::spin();

    return 0;
}
