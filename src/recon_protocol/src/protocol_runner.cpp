#include "protocol_runner.h"

ProtocolRunner::ProtocolRunner(ros::NodeHandle n)
{
    ProtocolRunner::nodeHandle = n;
};

bool ProtocolRunner::handleProtocolCall(recon_protocol::ProtocolInfo::Request &req, recon_protocol::ProtocolInfo::Response &res)
{
    ROS_INFO("recon protocol called!");
    res.Success = true;

    return true;
};

void ProtocolRunner::setup()
{
    ProtocolRunner::service = ProtocolRunner::nodeHandle.advertiseService("recon_protocol", &ProtocolRunner::handleProtocolCall, this);
}