#ifndef _CONTROLLERS_H_
#define _CONTROLLERS_H_

#include "pid.h"

class position_controller
{
public:
    position_controller(double min_vel = 0.0, double max_vel = 1.0, double min_pos = 0.0, double max_pos = 1.0, 
        double dt = 1.0, double Kpp = 0.1, double Kip = 0.01, double Kpv = 0.1, double Kiv = 0.01);
    ~position_controller();
    double calculate(double position_desired, double position_actual, double velocity_actual);
    void reset();
private:
    PID* position_pi;
    PID* position_derivator;
    PID* velocity_pi;

    double velocity_desired;
    double max_pos;
    double min_pos;
};

class velocity_controller
{
public:
    velocity_controller(double min = 0.0, double max = 1.0, double dt = 1.0, double Kpv = 0.1, double Kiv = 0.01);
    ~velocity_controller();
    double calculate(double velocity_desired, double velocity_actual);
    void reset();
private:
    PID* velocity_pi;
};

#endif // _CONTROLLERS_H_
