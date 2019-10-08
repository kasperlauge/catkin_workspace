#include "ros/ros.h"

#include "uav.h"

int main(int argv, char **argc)
{
    ros::init(argv, argc, "DroneLiftoff");

    ros::NodeHandle n;

    UAV drone(n);

    drone.run();

    return 0;
}