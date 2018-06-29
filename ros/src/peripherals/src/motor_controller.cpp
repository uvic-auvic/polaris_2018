#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>
#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "monitor/GetSerialDevice.h"
#include "peripherals/motor_enums.h"

#define NUM_MOTORS (8)
#define NUM_CHAR_PER_RPM (2)
#define NUM_CHAR_PER_PWM (2)
#define NUM_CHAR_PER_DIR (1)
#define MAX_MOTOR_VALUE (999)

using MotorReq = peripherals::motor::Request;
using MotorRes = peripherals::motor::Response;
using MotorsReq = peripherals::motors::Request;
using MotorsRes = peripherals::motors::Response;
using rosserv = ros::ServiceServer;

class motor_controller {
public:
    motor_controller(const std::string & port, int baud_rate = 9600, int timeout = 1000);
    ~motor_controller();
    bool setMotorPWM(MotorReq &req, MotorRes &res);
    bool setAllMotorsPWM(MotorsReq &req, MotorsRes &res);
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

    peripherals::motor_enums motor_defs;
}

motor_controller::~motor_controller() {
    this->write("STP");
    connection->close();
}

std::string motor_controller::write(const std::string & out, bool ignore_response, std::string eol)
{
    connection->flush();
    connection->write(out + eol);
    //ROS_INFO("%s", out.c_str());
    if (ignore_response) {
        return "";
    }
    return connection->readline(65536ul, eol);
}

bool motor_controller::setMotorPWM(MotorReq &req, MotorRes &res)
{
    int16_t pwm = req.pwm;
    std::string out = "M" + std::to_string(req.motor_num);
    std::string dir = "F";
    if(pwm < 0) {
        dir = "R";
        pwm *= -1;
    }
    if(pwm > MAX_MOTOR_VALUE) {
        pwm = MAX_MOTOR_VALUE;
    }
    out += dir;
    out += std::to_string((pwm/100) + '0');
    out += std::to_string(((pwm%100)/10) + '0');
    out += std::to_string((pwm%10) + '0');
    this->write(out);
    return true;
}

bool motor_controller::setAllMotorsPWM(MotorsReq &req, MotorsRes &res)
{
    char motor_num = '1';
    std::string out = "MSA";
    for (auto pwm : req.pwms) {
        char dir = 'F';
        if (pwm < 0) {
            dir = 'R';
            pwm *= -1;
        }
        if (pwm > MAX_MOTOR_VALUE) {
            pwm = MAX_MOTOR_VALUE;
        }
        out.push_back(dir); 
        out.push_back(((char)(pwm/100) + '0'));
        out.push_back(((char)((pwm%100)/10) + '0'));
        out.push_back(((char)(pwm%10) + '0'));
	motor_num++;
    }
    this->write(out);
    return true;
}

bool motor_controller::stopMotor(MotorReq &req, MotorRes &res)
{
    std::string out = "SM" + std::to_string(req.motor_num);
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
    if (rpm_string.size() != (NUM_CHAR_PER_PWM * NUM_MOTORS) + 2) { //assuming eol is \r\n
        return false;
    }

    for(uint8_t motor = 0; motor < NUM_MOTORS; motor++){
	res.rpms.push_back((int16_t)((rpm_string[(motor * 2) + 1] << 8) | (rpm_string[motor * 2])));
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
        ROS_ERROR("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Motor Controller on fd %s\n", srv.response.device_fd.c_str());

    motor_controller m(srv.response.device_fd);

    /* Setup all the Different services/commands which we  can call. Each service does its own error handling */
    rosserv mr  = nh.advertiseService("setMotorPWM", &motor_controller::setMotorPWM, &m);
    rosserv stp = nh.advertiseService("stopAllMotors", &motor_controller::stopAllMotors, &m);
    rosserv sm  = nh.advertiseService("stopMotor", &motor_controller::stopMotor, &m);
    rosserv rpm = nh.advertiseService("getRPM", &motor_controller::getRPM, &m);
    rosserv sam = nh.advertiseService("setAllMotorsPWM", &motor_controller::setAllMotorsPWM, &m);

    /* Wait for callbacks */
    ros::spin();

    return 0;
}

