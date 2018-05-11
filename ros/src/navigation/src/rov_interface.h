#ifndef ROV_INTERFACE_H
#define ROV_INTERFACE_H

#include <ros/ros.h>
#include "navigation/rov.h"

class rov_interface {
public:
    rov_interface() : new_msg(false) {}
    set_message(const navigation::rov &msg) {this->msg = msg;}

private:
    navigation::rov msg;
    bool new_msg;
}; 


#endif