#include "transformer.h"
#include <coordinate_transformation/CoordinateData.h>

Transformer::Transformer(ros::NodeHandle n)
{
    Transformer::nodeHandle = n;
};

bool Transformer::transformCoordinates(coordinate_transformation::CoordinateInfo::Request &req, coordinate_transformation::CoordinateInfo::Response &res)
{
    ROS_INFO("transform coordinates called!");

    return true;
};

void Transformer::setup()
{
    Transformer::service = Transformer::nodeHandle.advertiseService("transform_coordinates", &Transformer::transformCoordinates, this);
}