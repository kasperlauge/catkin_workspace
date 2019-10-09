#include "protocol_runner.h"

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "recon_protocol");
    ros::NodeHandle nh;

    ProtocolRunner protocolRunner(nh);
    protocolRunner.setup();

    ros::spin();

    return 0;
};