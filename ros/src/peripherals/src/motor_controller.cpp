#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include "motor_controller/motor_command.h"
//#include "system_manager/device_manager.h"

#define MAX_NUMBER_OF_MOTORS (8)
#define MAX_INDEX_OF_MOTORS (MAX_NUMBER_OF_MOTORS-1)


class motor_controller {
public:
    motor_controller(std::string port, int baud_rate, int timeout);
private:
    serial::Serial connection;;
    std::string write(std::string out, std::string eol, bool response);
};

motor_controller::motor_controller(std::string port, int baud_rate = 9600, timeout = 1000) {
    ROS_INFO("Connecting to motor_controller on port: %s", fd.c_str());
    connection = serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout));
}

std::string motor_controller::write(std::string out, std::string eol = "\r\n", bool response = true)
{
    connection.write(out + eol);
    if (!response) {
        return "";
    }

    return connection.readline(65536ul,"\r\n");
}

bool setMotorForward(motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: M${X}F${Arg}\n */
    std::string serial_out_str = "M" + std::to_string(req.motor_number) + "F" + std::string(1, (unsigned char) req.command_param) + "\n";

    /* Sending serial output */
    ROS_INFO("Sending serial output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, false);
    res.motor_success = parseSerialResponse(res.motor_response, false);
    // The point of the motor_success is to give the control_system a chance to re-adjust as necessary
    // Chances are good though that it will never be used
    // We can come back to this and see how the control system actually ends up being implemented

  return true;
}

bool setMotorReverse(motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: M${X}R${Arg}\n */
    std::string serial_out_str = "M" + std::to_string(req.motor_number) + "R" + std::string(1, (unsigned char) req.command_param) + "\n";

    /* Sending serial output */
    ROS_INFO("Sending serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, false);
    res.motor_success = parseSerialResponse(res.motor_response, false);

  return true;
}

bool stopAllMotors(  motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    ROS_INFO("Stopping all motors");
    /* String: STP\n */
    std::string serial_out_str = "STP\n" ;

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

    /* Default to true */
    res.motor_success = true;

  return true;
}

bool stopMotor(      motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: SM${X}\n */
    std::string serial_out_str = "SM" + std::to_string(req.motor_number) + "\n";

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

  return true;
}

bool getRPM(         motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: RV${X}\n */
    std::string serial_out_str = "RV" + std::to_string(req.motor_number) + "\n";

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

  return true;
}

bool setPWM(         motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: PW${X}{Arg}\n */
    std::string serial_out_str = "PW" + std::to_string(req.motor_number) + std::string(1, (unsigned char) req.command_param) + "\n";

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, false);
    res.motor_success = parseSerialResponse(res.motor_response, false);

  return true;
}

bool calibrateMotor( motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: CL${X}\n */
    std::string serial_out_str = "CL" + std::to_string(req.motor_number) + "\n";

    /* Sending serial output. Replace with actuall serial*/
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

  return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_con");
    ros::NodeHandle nh("~");

    // From System Manager, get the FD
    //ros::ServiceClient getDeviceName = nh.serviceClient<system_manager::device_manager>("/device_manager/getDeviceFd");
    //std::string sys_name;
    //nh.getParam("system_name", sys_name);
    //system_manager::device_manager srv;
    //srv.request.device = sys_name;
    //getDeviceName.call(srv);
    std::string fd;
    nh.getParam("motor_port", fd);
    setupSerialConnection(fd);

    /* Setup all the Different services/commands which we  can call. Each service does its own error handling */
    ros::ServiceServer setMotorForwardService = nh.advertiseService("setMotorForward", setMotorForward);
    ros::ServiceServer setMotorReverseService = nh.advertiseService("setMotorReverse", setMotorReverse);
    ros::ServiceServer stopAllMotorsService   = nh.advertiseService("stopAllMotors", stopAllMotors);
    ros::ServiceServer stopMotorService       = nh.advertiseService("stopMotors", stopMotor);
    ros::ServiceServer getRPMService          = nh.advertiseService("getRPM", getRPM);
    ros::ServiceServer setPWMService          = nh.advertiseService("setRPM", setPWM);
    ros::ServiceServer calibrateMotorService  = nh.advertiseService("calibrateMotor", calibrateMotor);

    /* Wait for callbacks */
    ros::spin();

    if (serial_conn != nullptr)
    {
        ROS_INFO("Closing connection on port %s", serial_conn->getPort().c_str());
        serial_conn->close();
        delete serial_conn;
    }

    return 0;
}
