#include "transformer.h"

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "coordinate_transformation");
    ros::NodeHandle nh;

    Transformer transformer(nh);
    transformer.setup();

    ros::spin();

    return 0;
};