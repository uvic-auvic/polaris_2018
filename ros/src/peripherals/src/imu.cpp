#include <ros/ros.h>
#include <string>
#include <memory>
#include <serial/serial.h>
#include <iostream>

#include "monitor/GetSerialDevice.h"
#include "peripherals/imu.h"
#include "peripherals/orientation.h"
#include "geometry_msgs/Vector3.h"

#define RESPONSE_MAX_SIZE (23)

#define READ_EEPROM_CMD         (0x28)
#define READ_EEPROM_SIZE        (7)
#define MAG_GAIN_SCALE_ADDR     (232)
#define ACCEL_GAIN_SCALE_ADDR   (230)
#define GYRO_GAIN_SCALE_ADDR    (130)

#define TEMP_CMD        (0x07)
#define TEMP_SIZE       (7)

#define EULER_STAB_CMD  (0x0E)
#define EULER_STAB_SIZE (11)

#define MAG_ACCEL_GYRO_STAB_CMD  (0x02)
#define MAG_ACCEL_GYRO_STAB_SIZE (23)

#define MAG_ACCEL_GYRO_CMD      (0x03)
#define MAG_ACCEL_GYRO_SIZE     (23)

using rosserv = ros::ServiceServer;
using imu_msg = peripherals::imu;

class imu{
public:
    imu(const std::string & port, int baud_rate = 38400, int timeout = 3000);
    ~imu();
    bool get_temperature(double &temperature);
    bool get_euler_stable(peripherals::orientation &euler_angles);
    bool get_mag_accel_gyro_stable(geometry_msgs::Vector3 &mag, geometry_msgs::Vector3 &accel, 
            geometry_msgs::Vector3 &gyro, double &time);
    bool get_mag_accel_gyro(geometry_msgs::Vector3 &mag, geometry_msgs::Vector3 &accel, 
            geometry_msgs::Vector3 &gyro, double &time);
    void get_velocity(geometry_msgs::Vector3 &velocity_vector);
    void update_velocity();
private:
    void write(uint8_t command, int response_bytes = 0);
    bool verify_response(int response_bytes);
    std::unique_ptr<serial::Serial> connection = nullptr;
    uint8_t * response_buffer = nullptr;
    double mag_gain_scale = 1;
    double accel_gain_scale = 1;
    double gyro_gain_scale = 1;
    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 last_accel;
    double last_timestamp;
};

imu::imu(const std::string & port, int baud_rate, int timeout) {
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;
    last_accel.x = 0.0;
    last_accel.y = 0.0;
    last_accel.z = 0.0;
    last_timestamp = 0.0;

    ROS_INFO("Connecting to imu on port: %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
    response_buffer = new uint8_t[RESPONSE_MAX_SIZE];

    // Need to get calibrated gain scales for magnetometer, gyroscope and accelerometer
    ROS_INFO("Acquiring the magnetometer gain scale.");
    write(READ_EEPROM_CMD);
    write(MAG_GAIN_SCALE_ADDR >> 8);
    write(MAG_GAIN_SCALE_ADDR & 0xFF, READ_EEPROM_SIZE);
    if(verify_response(READ_EEPROM_SIZE)) {     
        mag_gain_scale = ((int16_t) ((response_buffer[1] << 8) | response_buffer[2]));
    }
    else {      
        ROS_ERROR("Bad Checksum, failed to get MagGainScale");
    }

    ROS_INFO("Acquiring the accelerometer gain scale.");
    write(READ_EEPROM_CMD);
    write(ACCEL_GAIN_SCALE_ADDR >> 8);
    write(ACCEL_GAIN_SCALE_ADDR & 0xFF, READ_EEPROM_SIZE);
    if(verify_response(READ_EEPROM_SIZE)) {     
        accel_gain_scale = ((int16_t) ((response_buffer[1] << 8) | response_buffer[2]));
    }
    else {      
        ROS_ERROR("Bad Checksum, failed to get AccelGainScale");
    }

    ROS_INFO("Acquiring the gyroscope gain scale.");
    write(READ_EEPROM_CMD);
    write(GYRO_GAIN_SCALE_ADDR >> 8);
    write(GYRO_GAIN_SCALE_ADDR & 0xFF, READ_EEPROM_SIZE);
    if(verify_response(READ_EEPROM_SIZE)) {     
        gyro_gain_scale = ((int16_t) ((response_buffer[1] << 8) | response_buffer[2]));
    }
    else {      
        ROS_ERROR("Bad Checksum, failed to get GyroGainScale");
    }
    
    ROS_INFO("Gain scales: M:%f, A:%f, G:%f", mag_gain_scale, accel_gain_scale, gyro_gain_scale);
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

bool imu::verify_response(int response_bytes) {
    if((response_bytes > 0) && (response_bytes <= RESPONSE_MAX_SIZE)) {
        // Each response contains a checksum (last two bytes)
        uint16_t response_checksum = (response_buffer[response_bytes - 2] << 8) | (response_buffer[response_bytes - 1]);
        
        // Can compute the checksum by summing all bytes preceding the checksum as 16 bit numbers where
        // MSB of header is 0.
        uint16_t computed_checksum = response_buffer[0];
        for( int i = 1; i < (response_bytes - 2); i+=2) {    
            computed_checksum += (response_buffer[i] << 8) | (response_buffer[i+1]);
        }

        // Check if the checksums are the same
        return response_checksum == computed_checksum;
    }
    else {
        return false;
    }
}

bool imu::get_temperature(double &temperature) {
    // Send temperature command
    write(TEMP_CMD, TEMP_SIZE);

    // Verify the checksum of the response
    if(!verify_response(TEMP_SIZE)) {
        ROS_INFO("Bad checksum");
        return false;
    }
    
    // Compute temperature (first 2 non-header bytes of response)
    temperature = (( (double)(((int) response_buffer[1] << 8) | (int) response_buffer[2]) * 5.0 / 65536) - 0.5) * 100.0;

    return true;
}

bool imu::get_euler_stable(peripherals::orientation &euler_angles) {  
    // Send stable euler angles command
    write(EULER_STAB_CMD, EULER_STAB_SIZE);

    // Verify the checksum of the response
    if(!verify_response(EULER_STAB_SIZE)) {      
        ROS_INFO("Bad Checksum");
        return false;
    }

    euler_angles.roll = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * 360.0 / 65536.0;
    euler_angles.pitch = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * 360.0 / 65536.0;
    euler_angles.yaw = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * 360.0 / 65536.0;

    return true;
}

bool imu::get_mag_accel_gyro_stable
(
    geometry_msgs::Vector3 &mag, 
    geometry_msgs::Vector3 &accel, 
    geometry_msgs::Vector3 &gyro, 
    double &time
)
{
    // Send the stable vectors command
    write(MAG_ACCEL_GYRO_STAB_CMD, MAG_ACCEL_GYRO_STAB_SIZE);

    // Verify the checksum of the response
    if(!verify_response(MAG_ACCEL_GYRO_STAB_SIZE)) {      
        ROS_INFO("Bad Checksum");
        return false;
    }

    // Get magnetometer vector (Gauss)
    mag.x = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * mag_gain_scale / 32768000.0;
    mag.y = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * mag_gain_scale / 32768000.0;
    mag.z = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * mag_gain_scale / 32768000.0;

    // Get accelerometer vector (G's)
    accel.x = ((int16_t)((response_buffer[7] << 8) | response_buffer[8])) * accel_gain_scale / 32768000.0;
    accel.y = ((int16_t)((response_buffer[9] << 8) | response_buffer[10])) * accel_gain_scale / 32768000.0;
    accel.z = ((int16_t)((response_buffer[11] << 8) | response_buffer[12])) * accel_gain_scale / 32768000.0;

    // Get gyroscope vector (rad/sec)
    gyro.x = ((int16_t)((response_buffer[13] << 8) | response_buffer[14])) * gyro_gain_scale / 32768000.0;
    gyro.y = ((int16_t)((response_buffer[15] << 8) | response_buffer[16])) * gyro_gain_scale / 32768000.0;
    gyro.z = ((int16_t)((response_buffer[17] << 8) | response_buffer[17])) * gyro_gain_scale / 32768000.0;

    // Get timestamp (ms)
    time = ((uint16_t)((response_buffer[19] << 8) | response_buffer[20])) * 6.5536;

    return true;
}

bool imu::get_mag_accel_gyro
(
    geometry_msgs::Vector3 &mag, 
    geometry_msgs::Vector3 &accel, 
    geometry_msgs::Vector3 &gyro, 
    double &time
) 
{        
    // Send the command to get instantaneous vectors
    write(MAG_ACCEL_GYRO_CMD, MAG_ACCEL_GYRO_SIZE);

    // Verify the checksum of the response
    if(!verify_response(MAG_ACCEL_GYRO_SIZE)) {      
        ROS_INFO("Bad Checksum");
        return false;
    }

    // Get magnetometer vector (Gauss)
    mag.x = ((int16_t)((response_buffer[1] << 8) | response_buffer[2])) * mag_gain_scale / 32768000.0;
    mag.y = ((int16_t)((response_buffer[3] << 8) | response_buffer[4])) * mag_gain_scale / 32768000.0;
    mag.z = ((int16_t)((response_buffer[5] << 8) | response_buffer[6])) * mag_gain_scale / 32768000.0;

    // Get accelerometer vector (G's)
    accel.x = ((int16_t)((response_buffer[7] << 8) | response_buffer[8])) * accel_gain_scale / 32768000.0;
    accel.y = ((int16_t)((response_buffer[9] << 8) | response_buffer[10])) * accel_gain_scale / 32768000.0;
    accel.z = ((int16_t)((response_buffer[11] << 8) | response_buffer[12])) * accel_gain_scale / 32768000.0;

    // Get gyroscope vector (rad/sec)
    gyro.x = ((int16_t)((response_buffer[13] << 8) | response_buffer[14])) * gyro_gain_scale / 32768000.0;
    gyro.y = ((int16_t)((response_buffer[15] << 8) | response_buffer[16])) * gyro_gain_scale / 32768000.0;
    gyro.z = ((int16_t)((response_buffer[17] << 8) | response_buffer[17])) * gyro_gain_scale / 32768000.0;

    // Get timestamp (ms)
    time = ((uint16_t)((response_buffer[19] << 8) | response_buffer[20])) * 6.5536;

    return true;
}

void imu::update_velocity()
{     
    static bool valid_data = false;
    geometry_msgs::Vector3 dummy_v;
    double dummy_d;
    geometry_msgs::Vector3 gravity;
    geometry_msgs::Vector3 accel;
    double timestamp;

    // Get new readings
    get_mag_accel_gyro(dummy_v, accel, dummy_v, timestamp);
    get_mag_accel_gyro_stable(dummy_v, gravity, dummy_v, dummy_d);

    // Subtract gravity from acceleration to get true acceleration
    geometry_msgs::Vector3 accel_true;
    accel_true.x = accel.x - gravity.x;
    accel_true.y = accel.y - gravity.y;
    accel_true.z = accel.z - gravity.z;

    // Check if this is the first time the function was called
    if(!valid_data) {   
        last_accel = accel_true;
        last_timestamp = timestamp;
        valid_data = true;
        return;
    }

    // This chunk of code is necessary for timestamp overflows
    double this_time;
    // Check for timestamp overflow
    if(timestamp < last_timestamp) {    
        this_time = timestamp + 429490.2;
    }
    else {      
        this_time = timestamp;
    }

    // Compute the velocity by integrating the acceleration using trapezoid method of integration
    velocity.x = velocity.x + 9.8 * 0.5 * (accel_true.x - last_accel.x) * (this_time - last_timestamp) / 1000.0;
    velocity.y = velocity.y + 9.8 * 0.5 * (accel_true.y - last_accel.y) * (this_time - last_timestamp) / 1000.0;
    velocity.z = velocity.z + 9.8 * 0.5 * (accel_true.z - last_accel.z) * (this_time - last_timestamp) / 1000.0;

    // Update accel and timestamp
    last_accel = accel_true;
    last_timestamp = timestamp;
}

void imu::get_velocity(geometry_msgs::Vector3 &velocity_vector) {      
    velocity_vector = this->velocity;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    // Get the topic name and topic buffer size
    std::string topic_name;
    int topic_buffer_size;
    nh.getParam("topic_name", topic_name);
    nh.getParam("topic_buffer_size", topic_buffer_size);

    // Get the device id
    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    // Get loop rate parameters
    int publish_rate, updates_per_publish;
    nh.getParam("publish_rate", publish_rate);
    nh.getParam("updates_per_publish", updates_per_publish);

    // Get the port the device is on
    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if(!client.call(srv)) {     
        ROS_ERROR("Couldn't get \"%s\" file descriptor. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using IMU on fd \"%s\"", srv.response.device_fd.c_str());

    // Declare publisher
    ros::Publisher pub = nh.advertise<peripherals::imu>(topic_name, topic_buffer_size);

    // Update velocity "updates_per_publish" times for every topic publish
    imu dev(srv.response.device_fd);
    ros::Rate r(publish_rate * updates_per_publish);
    uint16_t count = 0;
    while(ros::ok()) {

        dev.update_velocity();

        if((++count % updates_per_publish) == 0) {
            peripherals::imu msg;
            bool valid_msg = true;

            count = 0;

            // Get the Temperature of the IMU
            valid_msg = dev.get_temperature(msg.temperature) && valid_msg;

            // Get the Stabilised Euler Angles
            valid_msg = dev.get_euler_stable(msg.euler_angles) && valid_msg;

            // Get the Stabilised IMU Sensor Vectors
            valid_msg = dev.get_mag_accel_gyro_stable(msg.stabilised_magnetic_field, 
                    msg.stabilised_acceleration, msg.compensated_angular_rate, msg.stabilised_vectors_timestamp) && valid_msg;

            // Get the Instantaneous IMU Sensor Vectors
            valid_msg = dev.get_mag_accel_gyro(msg.magnetic_field, msg.acceleration, msg.angular_rate, 
                    msg.instantaneous_vectors_timestamp) && valid_msg;

            // Get the velocity
            dev.get_velocity(msg.velocity);

            if(valid_msg) {   
                // Publish message
                pub.publish(msg);

                ROS_INFO("Temperature: %f", msg.temperature);
                ROS_INFO("Velocity: X:%f, Y:%f, Z:%f", msg.velocity.x, msg.velocity.y, msg.velocity.z);
                ROS_INFO("Euler Angles: P:%f, R:%f, Y:%f", msg.euler_angles.pitch, msg.euler_angles.roll, msg.euler_angles.yaw);
                ROS_INFO("Stabilised Mag: X:%f, Y:%f, Z:%f", msg.stabilised_magnetic_field.x, msg.stabilised_magnetic_field.y, 
                        msg.stabilised_magnetic_field.z);
                ROS_INFO("Stabilised Accel: X:%f, Y:%f, Z:%f", msg.stabilised_acceleration.x, msg.stabilised_acceleration.y, 
                        msg.stabilised_acceleration.z);
                ROS_INFO("Compensated Gyro: X:%f, Y:%f, Z:%f", msg.compensated_angular_rate.x, msg.compensated_angular_rate.y, 
                        msg.compensated_angular_rate.z);
                ROS_INFO("Stabilised Vector Timestamp: %f", msg.stabilised_vectors_timestamp);
                ROS_INFO("Mag: X:%f, Y:%f, Z:%f", msg.magnetic_field.x, msg.magnetic_field.y, 
                        msg.magnetic_field.z);
                ROS_INFO("Accel: X:%f, Y:%f, Z:%f", msg.acceleration.x, msg.acceleration.y, 
                        msg.acceleration.z);
                ROS_INFO("Gyro: X:%f, Y:%f, Z:%f", msg.angular_rate.x, msg.angular_rate.y, 
                        msg.angular_rate.z);
                ROS_INFO("Instantaneous Vector Timestamp: %f\n", msg.instantaneous_vectors_timestamp);
            }
            else {  
                ROS_ERROR("Invalid message.\n");
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

