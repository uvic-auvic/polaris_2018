#ifndef DEV_PROP_H
#define DEV_PROP_H

#include <string>
#include <exception>
#include <serial/serial.h>
#include "monitor/SerialDevice.h"
#include "monitor/GetSerialDevice.h"
#include "monitor/GetSerialDevices.h"

using string = std::string;
using ptree = boost::property_tree::ptree;
using MapPair = std::pair<std::string, monitor::SerialDevice>;
using SerialDeviceMap = std::map<std::string, monitor::SerialDevice>;
using GetSerialReq = monitor::GetSerialDevice::Request;
using GetSerialRes = monitor::GetSerialDevice::Response;
using GetSerialsReq = monitor::GetSerialDevices::Request;
using GetSerialsRes = monitor::GetSerialDevices::Response;

class DeviceNotFoundException : public std::runtime_error {
public:    
    DeviceNotFoundException(const std::string & dev) 
    :std::runtime_error("Could not find device \"" + dev + "\"")
    {}
};

class device_property {
public:
    device_property(
        string name, bool ignore, string ack_m, string ack_r, int baud, int timeout, 
        int retry_count, bool convert_to_bytes, size_t size_of_message, size_t size_of_response,
        bool big_endian_message, bool big_endian_response)
     : name(name)
     , ignore(ignore)
     , ack_message(ack_m)
     , ack_response(ack_r)
     , baud(baud)
     , convert_to_bytes(convert_to_bytes)
     , timeout(serial::Timeout::simpleTimeout(timeout))
     , retry_count(retry_count)
     , size_of_response(size_of_response)
     , size_of_message(size_of_message)
     , big_endian_message(big_endian_message)
     , big_endian_response(big_endian_response) {}
    std::string name;
    bool ignore;
    std::string ack_message;
    std::string ack_response;
    int baud;
    int retry_count;
    bool convert_to_bytes;
    size_t size_of_message;
    size_t size_of_response;
    serial::Timeout timeout;
    bool big_endian_message;
    bool big_endian_response;
};

class device_manager {
public:
    device_manager(const std::vector<device_property> & properties);
    bool get_all_devices(GetSerialsReq &, GetSerialsRes &);
    bool get_device_by_name(GetSerialReq &, GetSerialRes &);
private:
    SerialDeviceMap devices;
};

#endif /* DEV_PROP_H */
