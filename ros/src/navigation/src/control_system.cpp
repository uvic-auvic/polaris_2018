#include <ros/ros.h>
#include "navigation/nav.h"
#include "navigation/nav_request.h"
#include "navigation/control_en.h"
#include "navigation/depth_info.h"
#include "peripherals/imu.h"
#include "peripherals/powerboard.h"
#include "peripherals/avg_data.h"
#include "controllers.hpp"
#include "geometry_msgs/Vector3.h"
#include "median_filter.hpp"

#define ATMOSPHERIC_PRESSURE (1E5)

using AvgDataReq = peripherals::avg_data::Request;
using AvgDataRes = peripherals::avg_data::Response;
using ControlEnReq = navigation::control_en::Request;
using ControlEnRes = navigation::control_en::Response;

class control_system
{
public:
    control_system();
    ~control_system();
    void receive_nav_request(const navigation::nav_request::ConstPtr &msg);
    void receive_imu_data(const peripherals::imu::ConstPtr &msg);
    void receive_powerboard_data(const peripherals::powerboard::ConstPtr &msg);
    void compute_output_vectors(navigation::nav &msg);
    void populate_depth_data(navigation::depth_info &msg);
    bool calibrate_surface_depth(AvgDataReq &req, AvgDataRes &res);
    bool control_enable_service(ControlEnReq &req, ControlEnRes &res);
private:
    // ROS
    ros::NodeHandle nh;

    // Controllers
    position_controller* angular_pos_p;
    position_controller* angular_pos_r;
    velocity_controller* angular_vel_yw;
    velocity_controller* linear_vel_z;

    // Filters
    std::unique_ptr<filter_base> depth_filter;

    // Data
    navigation::nav_request::ConstPtr current_request;
    peripherals::imu::ConstPtr imu_data;
    double current_depth;
    double surface_pressure;
    bool depth_calibrated;

    // Enables
    ControlEnReq control_enables;
};

control_system::control_system():
    nh(ros::NodeHandle("~")),
    current_request(boost::shared_ptr<navigation::nav_request>(new navigation::nav_request())),
    imu_data(boost::shared_ptr<peripherals::imu>(new peripherals::imu())),
    current_depth(0),
    surface_pressure(ATMOSPHERIC_PRESSURE),
    depth_calibrated(false)
{
    control_enables.vel_x_enable = control_enables.vel_y_enable = control_enables.vel_z_enable = true;
    control_enables.pitch_enable = control_enables.roll_enable = control_enables.yaw_enable = true;

    // General Control System Parameters
    double loop_rate, min_lin_vel, max_lin_vel;
    double min_angl_vel, max_angl_vel, min_angl_pos, max_angl_pos;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("min_linear_vel", min_lin_vel);
    nh.getParam("max_linear_vel", max_lin_vel);
    nh.getParam("min_angular_vel", min_angl_vel);
    nh.getParam("max_angular_vel", max_angl_vel);
    nh.getParam("min_angular_pos", min_angl_pos);
    nh.getParam("max_angular_pos", max_angl_pos);
    double dt = 10.0 / loop_rate;
    
    // Veloctiy Z Control System
    double Kp_vel_z, Ki_vel_z;
    nh.getParam("Kp_vel_z", Kp_vel_z);
    nh.getParam("Ki_vel_z", Ki_vel_z);

    // Position Pitch Control System
    double Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p;
    nh.getParam("Kp_pos_p", Kp_pos_p);
    nh.getParam("Ki_pos_p", Ki_pos_p);
    nh.getParam("Kp_vel_p", Kp_vel_p);
    nh.getParam("Ki_vel_p", Ki_vel_p);

    // Position Roll Control System
    double Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r;
    nh.getParam("Kp_pos_r", Kp_pos_r);
    nh.getParam("Ki_pos_r", Ki_pos_r);
    nh.getParam("Kp_vel_r", Kp_vel_r);
    nh.getParam("Ki_vel_r", Ki_vel_r);

    // Velocity Yaw Control System
    double Kp_vel_yw, Ki_vel_yw;
    nh.getParam("Kp_vel_yw", Kp_vel_yw);
    nh.getParam("Ki_vel_yw", Ki_vel_yw);

    angular_pos_p = new position_controller(
            min_angl_vel, max_angl_vel, min_angl_pos, max_angl_pos, dt, Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p);
    angular_pos_r = new position_controller(
            min_angl_vel, max_angl_vel, min_angl_pos, max_angl_pos, dt, Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r);
    angular_vel_yw = new velocity_controller(min_angl_vel, max_angl_vel, dt, Kp_vel_yw, Ki_vel_yw);
    linear_vel_z = new velocity_controller(min_lin_vel, max_lin_vel, dt, Kp_vel_z, Ki_vel_z);

    int depth_filter_size;
    nh.getParam("depth_filter_size", depth_filter_size);
    depth_filter = std::unique_ptr<filter_base>(new median_filter(depth_filter_size));
}

control_system::~control_system()
{       
    delete linear_vel_z;
    delete angular_pos_p;
    delete angular_pos_r;
    delete angular_vel_yw;
}

void control_system::receive_nav_request(const navigation::nav_request::ConstPtr &msg) 
{      
    current_request = msg;
}

void control_system::receive_imu_data(const peripherals::imu::ConstPtr &msg)
{      
    imu_data = msg;
}

void control_system::receive_powerboard_data(const peripherals::powerboard::ConstPtr &msg)
{      
    // depth[m] = pressure[N/m^2] / (density[kg/m^3] * gravity[N/kg])
    constexpr float div = 997 * 9.81;
    //depth_filter->add_data((msg->external_pressure - surface_pressure) / div);
    //current_depth = depth_filter->get_result();
    current_depth = (msg->external_pressure - surface_pressure) / div;
}

bool control_system::calibrate_surface_depth(AvgDataReq &req, AvgDataRes &res)
{
    // Copy service message over and request average external pressure
    peripherals::avg_data srv;
    srv.request = req;
    ros::ServiceClient ext_pres_client = nh.serviceClient<peripherals::avg_data>("/power_board/AverageExtPressure");

    if(!ext_pres_client.call(srv))
    {
        ROS_ERROR("Failed to acquire external pressure data during depth calibration.");
        return false;
    }

    res = srv.response;

    // Update surface pressure with the average external pressure
    surface_pressure = srv.response.avg_data;
    depth_calibrated = true;
    return true;
}
    
bool control_system::control_enable_service(ControlEnReq &req, ControlEnRes &res)
{       
    this->control_enables = req;

    // Reset any control systems being disabled
    if(!control_enables.vel_z_enable)
    {
        linear_vel_z->reset();
    }
    if(!control_enables.pitch_enable)
    {
        angular_pos_p->reset();
    }
    if(!control_enables.roll_enable)
    {
        angular_pos_r->reset();
    }
    if(!control_enables.yaw_enable)
    {
        angular_vel_yw->reset();
    }

    return true;
}

void control_system::compute_output_vectors(navigation::nav &msg)
{       
    if(depth_calibrated)
    {
        if(control_enables.vel_x_enable)
        {
            msg.direction.x = current_request->forwards_velocity;
        }
        if(control_enables.vel_y_enable)
        {
            msg.direction.y = current_request->sideways_velocity;
        }
        if(control_enables.vel_z_enable)
        {
            msg.direction.z = linear_vel_z->calculate(current_request->depth, current_depth);
        }
        if(control_enables.pitch_enable)
        {
            msg.orientation.pitch = angular_pos_p->calculate(0, imu_data->euler_angles.pitch, imu_data->compensated_angular_rate.y);
        }
        if(control_enables.roll_enable)
        {
            msg.orientation.roll = angular_pos_r->calculate(0, imu_data->euler_angles.roll, imu_data->compensated_angular_rate.x);
        }
        if(control_enables.yaw_enable)
        {
            msg.orientation.yaw = angular_vel_yw->calculate(current_request->yaw_rate, imu_data->angular_rate.z);
        }
    }
    else
    {
        ROS_INFO("Please calibrate depth sensor.");
    }
}

void control_system::populate_depth_data(navigation::depth_info &msg)
{       
    msg.desired_depth = current_request->depth;
    msg.current_depth = current_depth;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "control_system");
    ros::NodeHandle nh("~");

    double loop_rate, max_lin_vel;
    nh.getParam("loop_rate", loop_rate);
    
    int loops_per_param_update;
    nh.getParam("loops_per_param_update", loops_per_param_update);

    control_system ctrl;

    ros::ServiceServer calib_depth = nh.advertiseService
        ("/nav/CalibrateSurfaceDepth", &control_system::calibrate_surface_depth, &ctrl);

    ros::ServiceServer ctrl_en = nh.advertiseService
        ("ControlSysEnable", &control_system::control_enable_service, &ctrl);

    ros::Publisher pub_vectors = nh.advertise<navigation::nav>("/nav/velocity_vectors", 1);

    ros::Publisher pub_ctrl_params = nh.advertise<navigation::depth_info>("/nav/depth_control_info", 1);

    ros::Subscriber sub_nav = nh.subscribe<navigation::nav_request>
        ("/nav/navigation", 1, &control_system::receive_nav_request, &ctrl);

    ros::Subscriber sub_imu = nh.subscribe<peripherals::imu>
        ("/imu/imu_sensor", 1, &control_system::receive_imu_data, &ctrl);

    ros::Subscriber sub_pbd = nh.subscribe<peripherals::powerboard>
        ("/power_board/power_board_data", 1, &control_system::receive_powerboard_data, &ctrl);

    ros::Rate r(loop_rate);
    uint8_t count = 0;
    while(ros::ok()) { 
        // Get the output vectors from the control system
        navigation::nav output_vectors;
        ctrl.compute_output_vectors(output_vectors);

        // Publish the vectors
        pub_vectors.publish(output_vectors);

        if(++count == loops_per_param_update)
        {
            count = 0;

            // Get the control system parameters
            navigation::depth_info parameters;
            ctrl.populate_depth_data(parameters);

            // Publish the control system parameters
            pub_ctrl_params.publish(parameters);
        }

        ros::spinOnce();
        r.sleep();
    }
}
