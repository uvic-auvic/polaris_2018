#include <ros/ros.h>
#include "controllers.hpp"
#include "navigation/thrusts.h"
#include "navigation/rpm_control_en.h"
#include "peripherals/rpms.h"
#include "peripherals/motor_enums.h"
#include "peripherals/motors.h"

#define NUM_MOTORS              (8)

#define MAX_FORWARD_PWM         (400)
#define MIN_FORWARD_PWM         (100)
#define MAX_REVERSE_PWM         (300)
#define MIN_REVERSE_PWM         (100)

using RpmCtrlEnReq = navigation::rpm_control_en::Request;
using RpmCtrlEnRes = navigation::rpm_control_en::Response;

class rpm_controller
{
public:
    rpm_controller();
    void receive_desired_rpms(const peripherals::rpms::ConstPtr &msg);
    void receive_actual_rpms(const peripherals::rpms::ConstPtr &msg);
    bool control_en(RpmCtrlEnReq &req, RpmCtrlEnRes &res);
    void compute_pwms(peripherals::motors &srv);
private:
    int16_t rpm_to_pwm(double rpm);

    // ROS
    ros::NodeHandle nh;

    // Controllers 
    std::vector<std::unique_ptr<velocity_controller>> thruster_controllers;

    // Messages
    std::vector<double> current_rpm_des;
    std::vector<double> current_rpm_act;
    std::vector<double> previous_rpm_des;

    // Variables
    std::vector<int16_t> pwms;
    bool control_sys_en;
};

rpm_controller::rpm_controller():
    nh(ros::NodeHandle("~")),
    current_rpm_des(NUM_MOTORS),
    current_rpm_act(NUM_MOTORS),
    previous_rpm_des(NUM_MOTORS),
    pwms(NUM_MOTORS),
    control_sys_en(false)
{
    double loop_rate, min_rpm, max_rpm, Kp, Ki;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("min_rpm", min_rpm);
    nh.getParam("max_rpm", max_rpm);
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);

    for(int i = 0; i < NUM_MOTORS; i++)
    {
        thruster_controllers.push_back(std::unique_ptr<velocity_controller>(new velocity_controller(min_rpm, max_rpm, 1.0/loop_rate, Kp, Ki)));
    }
}

int16_t rpm_controller::rpm_to_pwm(double rpm)
{
    //forward: rpm = 21.74167 (pwm) - 43.47222
    //reverse: rpm = 31.84857 (pwm) - 225.50476
    int16_t pwm = 0;
    if(rpm > 0)
    {
        pwm = (int16_t)((rpm + 43.47222) / 6.522501);
        if(pwm > MAX_FORWARD_PWM)
        {
            pwm = MAX_FORWARD_PWM;
        }
        else if(pwm < MIN_FORWARD_PWM)
        {
            pwm = 0;
        }
    }
    else
    {
        pwm = (int16_t)((-rpm + 225.50476) / 9.554571);
        if(pwm > MAX_REVERSE_PWM)
        {
            pwm = MAX_REVERSE_PWM;
        }
        else if(pwm < MIN_REVERSE_PWM)
        {
            pwm = 0;
        }
        pwm = -pwm;
    }

    return pwm;
}

void rpm_controller::compute_pwms(peripherals::motors &srv)
{
    // Get the required PWMs to correct the RPM
    for(int i = 0; i < NUM_MOTORS; i++)
    {
        // Compute the RPM
        double corrected_rpm = 0;
        if(control_sys_en)
        {
            // Determine the sign of the motor controller returned RPM
            double signed_rpm_des = (this->current_rpm_des[i] < 0) ? (-this->current_rpm_des[i]) : (this->current_rpm_des[i]);
            corrected_rpm = this->thruster_controllers[i]->calculate(signed_rpm_des, this->current_rpm_act[i]);
            corrected_rpm = (this->current_rpm_des[i] < 0) ? (-corrected_rpm) : (corrected_rpm);
        }
        else
        {
            corrected_rpm = this->current_rpm_des[i];
        }

        this->previous_rpm_des[i] = corrected_rpm;

        // Determine the desired PWM to achieve that RPM
        this->pwms[i] = this->rpm_to_pwm(corrected_rpm);
    }

    // Prepare service request
    srv.request.pwms = this->pwms;
}

bool rpm_controller::control_en(RpmCtrlEnReq &req, RpmCtrlEnRes &res)
{
    this->control_sys_en = req.enable;

    // If control system is being disabled, reset all controllers
    if(!req.enable)
    {   
        for(int i = 0; i < thruster_controllers.size(); i++)
        {       
            thruster_controllers[i]->reset();
        }
    }

    return true;
}

void rpm_controller::receive_desired_rpms(const peripherals::rpms::ConstPtr &msg)
{
    // RPMs must be in the correct order (see peripherals/msg/motor_enums.msg)
    current_rpm_des = msg->rpms;
}

void rpm_controller::receive_actual_rpms(const peripherals::rpms::ConstPtr &msg)
{
    // RPMs must be in the correct order (see peripherals/msg/motor_enums.msg)
    current_rpm_act = msg->rpms;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rpm_controller");
    ros::NodeHandle nh("~");
    rpm_controller rpm_ctrl;
    ros::Subscriber des_rpms = nh.subscribe<peripherals::rpms>("/nav/rpms", 1, &rpm_controller::receive_desired_rpms, &rpm_ctrl);
    ros::Subscriber act_rpms = nh.subscribe<peripherals::rpms>("/motor_controller/MotorsRPMs", 1, &rpm_controller::receive_actual_rpms, &rpm_ctrl);
    ros::ServiceClient mtrs_set_all = nh.serviceClient<peripherals::motors>("/motor_controller/setAllMotorsPWM");
    ros::ServiceServer control_en = nh.advertiseService("/nav/rpm_cntrl_en", &rpm_controller::control_en, &rpm_ctrl);
    ros::Publisher pub_pwms = nh.advertise<navigation::thrusts>("/rpm_control/pwms", 5);

    int loop_rate;
    nh.getParam("loop_rate", loop_rate);
    ros::Rate r(loop_rate);
    while(ros::ok())
    {
        // Compute the PWMs to from the RPM control systems
        peripherals::motors srv;
        rpm_ctrl.compute_pwms(srv);

        // Get PWMs
        navigation::thrusts msg;
        msg.thruster_pwms = srv.request.pwms;
        pub_pwms.publish(msg);

        // Set the motors
        mtrs_set_all.call(srv);
       
        // ROS stuff
        ros::spinOnce();
        r.sleep();
    }   

    return 0;
}
