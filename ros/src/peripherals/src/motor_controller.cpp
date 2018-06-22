#include <ros/ros.h>
#include <string>
#include <memory>
#include <map>
#include <serial/serial.h>
#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "monitor/GetSerialDevice.h"

#define NUM_MOTORS (8)
#define NUM_CHAR_PER_RPM (2)
#define NUM_CHAR_PER_PWM (2)
#define NUM_CHAR_PER_DIR (1)
#define MAX_MOTOR_VALUE (99)

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
    bool getMotorNames(MotorsReq &, MotorsRes &);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::map<std::string, uint8_t> motor_names_to_number;
    std::string write(const std::string & out, bool ignore_response = true, std::string eol = "\n");
};

motor_controller::motor_controller(const std::string & port, int baud_rate, int timeout) {
    ROS_INFO("Connecting to motor_controller on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));

    // Setup the motor name lookup dictionary
    motor_names_to_number["Z_Front_Right"] = 4;
    motor_names_to_number["Z_Front_Left"] = 3;
    motor_names_to_number["Z_Back_Right"] = 7;
    motor_names_to_number["Z_Back_Left"] = 6;
    motor_names_to_number["X_Right"] = 1;
    motor_names_to_number["X_Left"] = 8;
    motor_names_to_number["Y_Front"] = 2;
    motor_names_to_number["Y_Back"] = 5;
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
    int16_t pwm = req.motor_in.pwm;
    std::string out = "M" + std::to_string(motor_names_to_number[req.motor_in.name]);
    std::string dir = "F";
    if(pwm < 0) {
        dir = "R";
        pwm *= -1;
    }
    if(pwm > MAX_MOTOR_VALUE) {
        pwm = MAX_MOTOR_VALUE;
    }
    out += dir + std::to_string(pwm);
    this->write(out);
    res.motor_out.name = req.motor_in.name;
    res.motor_out.pwm = pwm * ((dir == "F") ? 1 : -1);
    return true;
}

bool motor_controller::setAllMotorsPWM(MotorsReq &req, MotorsRes &res)
{
    std::string out = "MSA";
    char* motor_pwms = new char[req.motors_in.size() * (NUM_CHAR_PER_PWM + NUM_CHAR_PER_DIR) + 1];
    motor_pwms[req.motors_in.size() * (NUM_CHAR_PER_PWM + NUM_CHAR_PER_DIR)] = '\0';
    for (auto motor : req.motors_in) {
        char dir = 'F';
        if (motor.pwm < 0) {
            dir = 'R';
            motor.pwm *= -1;
        }
        if (motor.pwm > MAX_MOTOR_VALUE) {
            motor.pwm = MAX_MOTOR_VALUE;
        }
        uint8_t index = motor_names_to_number[motor.name];
        motor_pwms[(index - 1) * (NUM_CHAR_PER_PWM + NUM_CHAR_PER_DIR)] = dir;
        std::strncpy(&(motor_pwms[1 + (index -1) * (NUM_CHAR_PER_PWM + NUM_CHAR_PER_DIR)]),
            std::to_string(motor.pwm).c_str(), NUM_CHAR_PER_PWM);
    }
    out += std::string(motor_pwms);
    this->write(out);
    delete[] motor_pwms;
    return true;
}

bool motor_controller::stopMotor(MotorReq &req, MotorRes &res)
{
    std::string out = "SM" + std::to_string(motor_names_to_number[req.motor_in.name]);
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

    std::map<std::string, uint8_t>::iterator it;
    for(it = motor_names_to_number.begin(); it != motor_names_to_number.end(); it++) {
        std::string rpm_i = rpm_string.substr((it->second - 1)*2, 2);
        peripherals::motor_info motor_i;

        // Get RPM for this motor
        motor_i.name = it->first;
        motor_i.rpm = (int16_t)((rpm_i[1] << 8) | (rpm_i[0]));

        // Add RPM to response
        res.motors_out.push_back(motor_i);
    }

    return true;
}

bool motor_controller::getMotorNames(MotorsReq &req, MotorsRes &res)
{
    std::map<std::string, uint8_t>::iterator it;
    for(it = motor_names_to_number.begin(); it != motor_names_to_number.end(); it++) {
        peripherals::motor_info motor_i;
        motor_i.name = it->first;
        res.motors_out.push_back(motor_i);
    }
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
    rosserv nms = nh.advertiseService("getMotorNames", &motor_controller::getMotorNames, &m);
    rosserv sam = nh.advertiseService("setAllMotorsPWM", &motor_controller::setAllMotorsPWM, &m);

    /* Wait for callbacks */
    ros::spin();

    return 0;
}

