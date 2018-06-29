#include <ros/ros.h>
#include "navigation/nav.h"
#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "peripherals/motor_enums.h"

#define Motor_Num               (8)

#define MAX_FORWARD_RPM         (3000.0)
#define RPM_FORWARD_SQ_COEFF    (0.00000389750967963493)
#define RPM_SCALE_FORWARD       (MAX_FORWARD_RPM / sqrt(1.0 / RPM_FORWARD_SQ_COEFF))

#define MAX_REVERSE_RPM         (2500.0)
#define RPM_REVERSE_SQ_COEFF    (0.00000396914500683942)
#define RPM_SCALE_REVERSE       (MAX_REVERSE_RPM / sqrt(1.0 / RPM_REVERSE_SQ_COEFF))

#define MAX_FORWARD_COMMAND     (400) //37 original
#define MIN_FORWARD_COMMAND     (100)
#define MAX_REVERSE_COMMAND     (300) //28 original
#define MIN_REVERSE_COMMAND     (100)

#define E_MATRIX_ROWS           (8)
#define E_MATRIX_COLUMNS        (6)

#define Z_FRONT_LEFT_POS        (0)
#define Z_FRONT_RIGHT_POS       (1)
#define Z_BACK_LEFT_POS         (2)
#define Z_BACK_RIGHT_POS        (3)
#define Y_FRONT_POS	        (4)
#define Y_BACK_POS	        (5)
#define X_LEFT_POS	        (6)
#define X_RIGHT_POS	        (7)



/*
using MotorReq = peripherals::motor::Request;
using MotorRes = peripherals::motor::Response;
using MotorsReq = peripherals::motors::Request;
using MotorsRes = peripherals::motors::Response;
*/

class thrust_controller
{
public:
    thrust_controller(std::string node_name);
    void generate_thrust_val(const navigation::nav::ConstPtr &msg);
    void do_thrust_matrix(double tau[E_MATRIX_COLUMNS], double thrust_value[]);
private:
    ros::NodeHandle nh;
    ros::ServiceClient motor_forward;
    ros::ServiceClient motor_reverse;
    ros::ServiceClient setAllMotorsPWM;
    ros::ServiceClient motor_stop;
    ros::ServiceClient motors_stop;

    int16_t thrust_to_command(double thrust);
    
    /*
    Top looking down view:
    For calculating system E only

          Front
            4
        0       1
        6       7
        2       3
            5
    */

    //motor order = 1z, 2z, 3z, 4z, 1y, 2y, 1x, 2x
    const double E_inverse[E_MATRIX_ROWS][E_MATRIX_COLUMNS] = {
        {0.0355383007316710, 0.0626175759204514, 0.226381127185469, -0.0223954134193317, -0.0186650739136927, -1.52872506320456e-17},
        {0.0355383007316709, -0.0626175759204520, 0.224903029899793, 0.0223954134193317, -0.0186650739136927, -1.61546123700340e-17},
        {-0.0355383007316710, 0.0626175759204510, 0.275096970100207, -0.0223954134193317, 0.0186650739136927, -1.68051336735253e-17},
        {-0.0355383007316710, -0.0626175759204516, 0.273618872814531, 0.0223954134193317, 0.0186650739136927, -1.03270256929244e-17},
        {0.000627995739502264, 0.475165623028772, 3.45847021392878e-17, 7.96797484263443e-17, 3.25599464924553e-18, 0.0190301739243126},
        {-0.000627995739502344, 0.524834376971227, -4.03195175199153e-17, 9.52532044474495e-17, 2.26581313390525e-18, -0.0190301739243126},
        {0.500349415164236, -0.0138177814947827, -1.91896838471135e-16, -1.39889909101611e-17, -1.04083408558608e-16, 0.0105883383101780},
        {0.499650584835764, 0.0138177814947827, 1.17507691233554e-16, -2.83689203505640e-17, -1.01047642475649e-16, -0.0105883383101780}
    };

}; // end class thrust_controller

void thrust_controller::do_thrust_matrix(double tau[E_MATRIX_COLUMNS], double thrust_value[Motor_Num]){
    // Thrusters = (E^-1) * tau
    for(int r = 0; r < E_MATRIX_ROWS; r++){
        thrust_value[r] = 0;
        for(int c = 0; c < E_MATRIX_COLUMNS; c++){
            thrust_value[r] += E_inverse[r][c] * tau[c];
        }
    }
}

int16_t thrust_controller::thrust_to_command(double thrust){
    //command use to max at 300 now maxes at 100. both cover same range the command = command * 3

    //forward: rpm = 21.74167 (command) - 43.47222
    //forward: thrust = (x^2) * 0.00000389750967963493  x is rpm, thrust is in newtons

    //reverse: rpm = 31.84857 (command) - 225.50476
    //reverse: thrust = (x^2) * -0.00000396914500683942  x is rpm, thrust is in newtons

    int command = 0;
    if(thrust > 0){
        
        unsigned int rpm = sqrt(thrust / RPM_FORWARD_SQ_COEFF);
        rpm = RPM_SCALE_FORWARD * rpm;
        command = (int)((rpm + 43.47222) / 6.522501);

        if(command > MAX_FORWARD_COMMAND){
            command = MAX_FORWARD_COMMAND;
        }else if(command < MIN_FORWARD_COMMAND){
            command = 0;
        }
    }else{
        unsigned int rpm = sqrt((-thrust) / RPM_REVERSE_SQ_COEFF);
        rpm = rpm * RPM_SCALE_REVERSE;
        command = (int)((rpm + 225.50476) / 9.554571);
        
        if(command > MAX_REVERSE_COMMAND){
            command = MAX_REVERSE_COMMAND;
        }else if(command < MIN_REVERSE_COMMAND){
            command = 0;
        }

        command = -command;
    }

    return command;
}

thrust_controller::thrust_controller(std::string node_name) :
    nh(ros::NodeHandle("~")),
    motor_forward(nh.serviceClient<peripherals::motor>("/" + node_name + "/setMotorForward")),
    motor_reverse(nh.serviceClient<peripherals::motor>("/" + node_name + "/setMotorReverse")),
    setAllMotorsPWM(nh.serviceClient<peripherals::motors>("/" + node_name + "/setAllMotorsPWM")),
    motor_stop(nh.serviceClient<peripherals::motor>("/" + node_name + "/stopMotors")),
    motors_stop(nh.serviceClient<peripherals::motor>("/" + node_name + "/stopAllMotors"))
    {
	
    }

void thrust_controller::generate_thrust_val(const navigation::nav::ConstPtr &msg)
{
    double tau[E_MATRIX_COLUMNS] = {
        msg->direction.x, 
        msg->direction.y, 
        msg->direction.z, 
        msg->orientation.pitch,
        msg->orientation.roll,
        msg->orientation.yaw
    };   
    double thruster_vals[Motor_Num] = {0.0};
    this->do_thrust_matrix(tau, thruster_vals);

    std::vector<int16_t> pwms(Motor_Num);

    pwms[peripherals::motor_enums::X_Left - 1] = this->thrust_to_command(thruster_vals[X_LEFT_POS]);
    pwms[peripherals::motor_enums::X_Right - 1] = this->thrust_to_command(thruster_vals[X_RIGHT_POS]);
    pwms[peripherals::motor_enums::Y_Front - 1] = this->thrust_to_command(thruster_vals[Y_FRONT_POS]);
    pwms[peripherals::motor_enums::Y_Back - 1] = this->thrust_to_command(thruster_vals[Y_BACK_POS]);
    pwms[peripherals::motor_enums::Z_Front_Right - 1] = this->thrust_to_command(thruster_vals[Z_FRONT_RIGHT_POS]);
    pwms[peripherals::motor_enums::Z_Front_Left - 1] = this->thrust_to_command(thruster_vals[Z_FRONT_LEFT_POS]);
    pwms[peripherals::motor_enums::Z_Back_Right - 1] = this->thrust_to_command(thruster_vals[Z_BACK_RIGHT_POS]);
    pwms[peripherals::motor_enums::Z_Back_Left - 1] = this->thrust_to_command(thruster_vals[Z_BACK_LEFT_POS]);
    
    peripherals::motors srv;
    srv.request.pwms = pwms;

    this->setAllMotorsPWM.call(srv.request, srv.response);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "thrustmap");
    ros::NodeHandle nh("~");
    thrust_controller tc("motor_controller");
    ros::Subscriber joy = nh.subscribe<navigation::nav>
        ("/nav/navigation", 1, &thrust_controller::generate_thrust_val, &tc);
    ros::spin();
    return 0;
}
