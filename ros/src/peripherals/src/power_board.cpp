#include <ros/ros.h>

#include <string>
#include <memory>
#include <serial/serial.h>

#include "monitor/GetSerialDevice.h"
#include "peripherals/powerboard.h"
#include "peripherals/power_enable.h"
#include "peripherals/avg_data.h"

#define RESPONSE_SIZE_CRA (14)
#define RESPONSE_SIZE_VTA (6)
#define RESPONSE_SIZE_TMP (4)
#define RESPONSE_SIZE_HUM (4)
#define RESPONSE_SIZE_WTR (4)
#define RESPONSE_SIZE_PIN (5)
#define RESPONSE_SIZE_PEX (4)

#define PEX_TO_PASCAL_MUL (68.94757)
#define RETRY_COUNT (3)

using rosserv = ros::ServiceServer;
using powerboardInfo = peripherals::powerboard;
using PowerEnableReq = peripherals::power_enable::Request;
using PowerEnableRes = peripherals::power_enable::Response;
using AvgDataReq = peripherals::avg_data::Request;
using AvgDataRes = peripherals::avg_data::Response;

class power_board{
public:
    power_board(const std::string & port, int baud_rate = 115200, int timeout = 1000);
    ~power_board();
    bool get_powerboard_data(powerboardInfo & msg);
    bool power_enabler(PowerEnableReq &req, PowerEnableRes &res);
    bool average_ext_pressure(AvgDataReq &req, AvgDataRes &res);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
    std::string write(const std::string & out, bool ignore_response = true, std::string eol = "\n");
    std::size_t write(const std::string & out, uint8_t* in, std::size_t read_len, std::string eol = "\n");
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
    // Flush the output, then the input (order matters, to flush any unwanted responses)
    connection->flushOutput();
    connection->flushInput();

    // Send command 
    connection->write(out + eol);
    //ROS_INFO("%s", out.c_str());

    // Used for commands where response does not matter (enabling, etc)
    if (ignore_response) {
        return "";
    }

    // Used for commands to read data
    return connection->readline(65536ul, eol);
}

std::size_t power_board::write(const std::string & out, uint8_t* in, std::size_t read_len, std::string eol)
{
    // Flush the output, then the input (order matters, to flush any unwanted responses)
    connection->flushOutput();
    connection->flushInput();

    // Send command 
    connection->write(out + eol);
    //ROS_INFO("%s", out.c_str());

    // Read specified data length
    return connection->read(in, read_len);
}

bool power_board::get_powerboard_data(powerboardInfo &msg) {
    // Get data from power board
    std::unique_ptr<uint8_t[]> currents(new uint8_t[RESPONSE_SIZE_CRA]);
    std::unique_ptr<uint8_t[]> voltages(new uint8_t[RESPONSE_SIZE_VTA]);
    std::unique_ptr<uint8_t[]> temperature(new uint8_t[RESPONSE_SIZE_TMP]);
    std::unique_ptr<uint8_t[]> humidity(new uint8_t[RESPONSE_SIZE_HUM]);
    std::unique_ptr<uint8_t[]> water(new uint8_t[RESPONSE_SIZE_WTR]);
    std::unique_ptr<uint8_t[]> pressure_internal(new uint8_t[RESPONSE_SIZE_PIN]);
    std::unique_ptr<uint8_t[]> pressure_external(new uint8_t[RESPONSE_SIZE_PEX]);

    // Populate the message with current data
    if( (this->write("CRA", currents.get(), RESPONSE_SIZE_CRA) == RESPONSE_SIZE_CRA) &&
        (std::memcmp(&(currents[RESPONSE_SIZE_CRA-2]), "\r\n", 2) == 0)) 
    {       
        msg.current_battery_1 = (currents[2] << 16) | (currents[1] << 8) | (currents[0]);
        msg.current_battery_2 = (currents[5] << 16) | (currents[4] << 8) | (currents[3]);
        msg.current_motors = (currents[8] << 16) | (currents[7] << 8) | (currents[6]);
        msg.current_system = (currents[11] << 16) | (currents[10] << 8) | (currents[9]);
    }
    else {      
        ROS_ERROR("Current data is invalid.");
        return false;
    }

    // Populate message with voltage data
    if( (this->write("VTA", voltages.get(), RESPONSE_SIZE_VTA) == RESPONSE_SIZE_VTA) &&
        (std::memcmp(&(voltages[RESPONSE_SIZE_VTA-2]), "\r\n", 2) == 0))
    {        
        msg.voltage_battery_1 = (voltages[1] << 8) | (voltages[0]);
        msg.voltage_battery_2 = (voltages[3] << 8) | (voltages[2]);
    }
    else {      
        ROS_ERROR("Voltage data is invalid.");
        return false;
    }

    // Populate message with temperature data
    if( (this->write("TMP", temperature.get(), RESPONSE_SIZE_TMP) == RESPONSE_SIZE_TMP) &&
        (std::memcmp(&(temperature[RESPONSE_SIZE_TMP-2]), "\r\n", 2) == 0))
    {
        // Convert to degrees C
        msg.temperature = (temperature[1] << 8) | (temperature[0]);
        msg.temperature = (msg.temperature / 10.0) - 273.15;
    }
    else {      
        ROS_ERROR("Temperature data is invalid.");
        return false;
    }

    // Populate message with humidity data
    if( (this->write("HUM", humidity.get(), RESPONSE_SIZE_HUM) == RESPONSE_SIZE_HUM) &&
        (std::memcmp(&(humidity[RESPONSE_SIZE_HUM-2]), "\r\n", 2) == 0))
    {
        msg.humidity = (humidity[1] << 8) | (humidity[0]);
    }
    else {      
        ROS_ERROR("Humidity data is invalid.");
        return false;
    }

    // Populate message with water sensor data
    if( (this->write("WTR", water.get(), RESPONSE_SIZE_WTR) == RESPONSE_SIZE_WTR) &&
        (std::memcmp(&(water[RESPONSE_SIZE_WTR-2]), "\r\n", 2) == 0))
    {
        msg.water_sensor = (water[1] << 8) | (water[0]);
    }
    else {      
        ROS_ERROR("Water data is invalid.");
        return false;
    }

    // Populate message with main housing pressure data
    if( (this->write("PIN", pressure_internal.get(), RESPONSE_SIZE_PIN) == RESPONSE_SIZE_PIN) &&
        (std::memcmp(&(pressure_internal[RESPONSE_SIZE_PIN-2]), "\r\n", 2) == 0))
    {
        msg.internal_pressure = (pressure_internal[2] << 16) | (pressure_internal[1] << 8) | (pressure_internal[0]);
        // No conversions needed, pressure is in Pa
    }
    else {      
        ROS_ERROR("Internal housing pressure data is invalid."); 
        return false;
    }

    // Populate message with external water pressure data
    if( (this->write("PEX", pressure_external.get(), RESPONSE_SIZE_PEX) == RESPONSE_SIZE_PEX) &&
        (std::memcmp(&(pressure_external[RESPONSE_SIZE_PEX-2]), "\r\n", 2) == 0))
    {
        msg.external_pressure = (pressure_external[1] << 8) | (pressure_external[0]);
        // Convert from 0.01psi to Pa
        msg.external_pressure *= PEX_TO_PASCAL_MUL;
    }
    else {      
        ROS_ERROR("External water pressure data is invalid."); 
        return false;
    }

    return true;
}

bool power_board::power_enabler(PowerEnableReq &req, PowerEnableRes &res)
{      
    // Command structure: PXEb -> X is the system, b is either 0 or 1
    // X can be: M for motors, 5 for 5V, T or 9 for the 9V/12V rail, or S for system
    // Enable/Disable Power to Motors
    std::string out = "PME" + std::string(req.motor_pwr_enable ? "1" : "0");
    write(out);

    // Enable/Disable 5V Rail 
    out.replace(1, 1, "5"); // Replace X position with 5 for 5V rail
    out.replace(3, 1, req.rail_5V_pwr_enable ? "1" : "0"); // Replace b position with either 0 or 1
    write(out);

    // Enable/Disable 12V/9V Rails
    out.replace(1, 1, "T"); // Replace X position with T for 9V/12V rail
    out.replace(3, 1, req.rail_12V_9V_pwr_enable ? "1" : "0"); // Replace b position with either 0 or 1
    write(out);

    // Command structure: BPb -> b is either 0 or 1 
    // Enable/Disable Running Batteries in Parallel 
    out = "BP" + std::string(req.parallel_batteries_enable ? "1" : "0");
    write(out);

    return true;
}

bool power_board::average_ext_pressure(AvgDataReq &req, AvgDataRes &res)
{
    // Use rate to determine read speed
    ros::Rate r(req.acq_rate);

    // Try and acquire the appropriate amount of data
    int retry_count = 0;
    res.avg_data = 0;
    uint8_t pex_response[RESPONSE_SIZE_PEX];
    for(int i = 0; i < req.acq_count; i++)
    {
        // Read the external pressure, and add to average sum.
        if(this->write("PEX", pex_response, RESPONSE_SIZE_PEX) == RESPONSE_SIZE_PEX) {
            double external_pressure = (pex_response[1] << 8) | (pex_response[0]);
            res.avg_data += external_pressure / req.acq_count;
            r.sleep();
        }

        // Retry if read failed
        else if(retry_count < RETRY_COUNT) {
            ROS_ERROR("Failed to read pressure. Retry Count:%d", retry_count);
            i--;
        }

        // If number of retries exceeded, fail the service call
        else {
            ROS_ERROR("Failed to read pressure within retry count. Service request failed.");
            return false;
        }
    }

    // Convert pressure to Pascals
    res.avg_data *= PEX_TO_PASCAL_MUL;

    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "power_board");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    int loop_rate;
    nh.getParam("loop_rate", loop_rate);

    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv)) {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Power Board on fd %s\n", srv.response.device_fd.c_str());
    power_board device(srv.response.device_fd);

    ros::Publisher pub = nh.advertise<peripherals::powerboard>("power_board_data", 1);
    ros::ServiceServer pwr_en = nh.advertiseService("PowerEnable", &power_board::power_enabler, &device); 
    ros::ServiceServer avg_ext_p = nh.advertiseService("AverageExtPressure", &power_board::average_ext_pressure, &device);

    // Main loop
    ros::Rate r(loop_rate);
    while(ros::ok()) {
        // Publish message to topic 
        peripherals::powerboard msg;
        if(device.get_powerboard_data(msg)) {
            pub.publish(msg);
        }

        // End of loop maintenance
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

