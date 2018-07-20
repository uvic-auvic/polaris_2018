#include "controllers.hpp"
#include <ros/ros.h>

/* ------ Position Controller ------ */

position_controller::position_controller(double min_vel, double max_vel, double min_pos, double max_pos,
        double dt, double Kpp, double Kip, double Kpv, double Kiv):
    velocity_desired(0),
    min_pos(min_pos),
    max_pos(max_pos)
{
    // Initialize PI controller for position
    this->position_pi = new PID(dt, max_pos, min_pos, Kpp, 0.0, Kip);

    // Initialize D controller to take derivative of position to get velocity
    this->position_derivator = new PID(dt, max_vel, min_vel, 0.0, 1.0, 0.0);

    // Initialize PI controller for velocity
    this->velocity_pi = new PID(dt, max_vel, min_vel, Kpv, 0.0, Kiv);
}

position_controller::~position_controller()
{
    delete this->position_pi;
    delete this->position_derivator;
    delete this->velocity_pi;
}

double position_controller::calculate(double position_desired, double position_actual, double velocity_actual)
{
    // Get PI result from positional PI controller
    double position_correction = this->position_pi->calculate(position_desired, position_actual);

    // Take derivative of positional correction to get a velocity 
    if(position_correction > min_pos && position_correction < max_pos)
    {
        velocity_desired = this->position_derivator->calculate(position_correction, 0);
    }

    // Compute a velocity correction
    return this->velocity_pi->calculate(velocity_desired, velocity_actual);
}

void position_controller::reset()
{       
    this->velocity_desired = 0;
    this->position_pi->reset();
    this->position_derivator->reset();
    this->velocity_pi->reset();
}

/* ------ Velocity Controller ------ */

velocity_controller::velocity_controller(double min, double max, double dt, double Kpv, double Kiv)
{
    // Initialize PI controller for velocity
    this->velocity_pi = new PID(dt, max, min, Kpv, 0.0, Kiv);
}

velocity_controller::~velocity_controller()
{
    delete this->velocity_pi;
}

double velocity_controller::calculate(double velocity_desired, double velocity_actual)
{
    // Compute the velocity correction
    return this->velocity_pi->calculate(velocity_desired, velocity_actual);
}

void velocity_controller::reset()
{       
    velocity_pi->reset(); 
}
