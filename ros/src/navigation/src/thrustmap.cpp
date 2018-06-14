#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "navigation/ThrustAction.h"
#include "navigation/nav.h"

/*  The matrix E was precomputed in Matlab/Octave then dumped here. 
    The method, summarized below, can be found in Chris Carpenter's Report
    ..
*/

const float e_matrix[3][7] = {
        {0.35355, 0.35292, 0.35355, 0.35292, 0.00141, 0.00141, 0.02099},
        {0.35355, -0.35292, 0.35355, -0.35292, -0.00141, -0.00141, -0.02099},
        {-0.00000, 0.00199, 0.00000, 0.00199, 0.49554, 0.49554, -0.06646}};

class thrust_mapper 
{
public:
    thrust_mapper();
    void generate_thrust_val(const navigation::ThrustGoalConstPtr &goal);
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<navigation::ThrustAction> as;
};

thrust_mapper::thrust_mapper() :
    nh("~"),
    as(nh, "thrustmap", boost::bind(&thrust_mapper::generate_thrust_val, this, _1), false)
{
    as.start();
}

void thrust_mapper::generate_thrust_val(const navigation::ThrustGoalConstPtr &goal)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrustmap");
    ros::NodeHandle nh("~");
    thrust_mapper m;
    ros::spin();
    return 0;
}