#include "SerialInterface.h"
#include "wmj_link.h"


SerialInterface::SerialInterface()
{
    SerialPort = boost::make_shared<SerialNode>("serial:///dev/ttyUSB0");
    node_thread = boost::thread(boost::bind(&SerialNode::mainRun, SerialPort));
    node_thread.detach();
}
SerialInterface::~SerialInterface()
{
}
std::vector<float> SerialInterface::readGimbal()
{
    // std::vector<float> result(2);
	// result[0] = my_robot.measure_gimbal_pose.yaw;
	// result[1] = my_robot.measure_gimbal_pose.pitch;
    // return result;
    return SerialPort->readGimbal();
}
void SerialInterface::setGimbal(float yaw, float pitch)
{
    // my_robot.expect_gimbal_pose.pitch = pitch;
    // my_robot.expect_gimbal_pose.yaw = yaw;
    SerialPort->setGimbal(yaw,pitch);
}
void SerialInterface::setShoot(bool shoot)
{
    // my_robot.is_shooting = is_shooting_;
    SerialPort->setShoot(shoot);
}