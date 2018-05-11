#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <iostream>

#include "monitor/SerialDevice.h"
#include "monitor/GetSerialDevice.h"
#include "monitor/GetSerialDevices.h"

using MapPair = std::pair<std::string, monitor::SerialDevice>;
using SerialDeviceMap = std::map<std::string, monitor::SerialDevice>;
using GetSerialReq = monitor::GetSerialDevice::Request;
using GetSerialRes = monitor::GetSerialDevice::Response;
using GetSerialsReq = monitor::GetSerialDevices::Request;
using GetSerialsRes = monitor::GetSerialDevices::Response;

class device_manager {
public:
    device_manager(int, int);
    bool get_all_devices(GetSerialsReq &, GetSerialsRes &);
    bool get_device_by_name(GetSerialReq &, GetSerialRes &);
private:
    SerialDeviceMap devices;
};

device_manager::device_manager(int baud_rate = 9600, int timeout = 1000) {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

	for(auto device = devices_found.begin(); device != devices_found.end(); ++device) {
        std::string serial_usb_prefix("/dev/ttyUSB");
        // Lots of other tty's turn up here, we just want /dev/TTYUSBX which are USB devices
        if (!std::equal(serial_usb_prefix.begin(), serial_usb_prefix.end(), device->port.begin())) {
            continue;
        }

        ROS_INFO("Attempting to open %s\n", device->port.c_str());
        serial::Serial connection(device->port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout));
        
        // Get RID, create SerialDevice instance and add to map
        monitor::SerialDevice dev;
        connection.write("RID\n");
        // Strip \r\n (last 2 chars). Probably a better way to do it
        std::string raw_string = connection.readline(65536ul, "\n");
        dev.name = raw_string.erase(raw_string.size() - 2);
        dev.port = device->port;
        devices.insert(MapPair(dev.name, dev));

        // Close connection
        ROS_INFO("Connected to device \"%s\" on %s\n", dev.name.c_str(), dev.port.c_str());
        connection.close(); 
	}
}

bool device_manager::get_device_by_name(GetSerialReq &req, GetSerialRes &res) {

    SerialDeviceMap::iterator it = devices.find(req.device_id);
    if (it == devices.end()) {
        ROS_INFO("No Device with name \"%s\"\n", req.device_id.c_str());
        return false;
    }

    ROS_INFO("found Device with name \"%s\"\n", req.device_id.c_str());
    res.device_fd = it->second.port;
    return true;
}

bool device_manager::get_all_devices(GetSerialsReq &req, GetSerialsRes &res) {
    //std::vector<monitor::SerialDevice> v;
    std::transform(devices.begin(), devices.end(), std::back_inserter(res.devices), 
        [](const MapPair &p) { 
            ROS_INFO(" %s : %s \n", p.second.name.c_str(), p.second.port.c_str());
            return p.second; });
    return true;
}
                                
int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "device_manager");
    ros::NodeHandle nh("~");

    device_manager m;

    ros::ServiceServer getDevice = nh.advertiseService("GetDevicePort", &device_manager::get_device_by_name, &m);
    ros::ServiceServer getDevices = nh.advertiseService("GetAllDevices", &device_manager::get_all_devices, &m);
    ros::spin();
    
    return 0;
}