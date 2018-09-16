#include "SerialNode.h"
#include "robot_abstract.h"
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

class SerialInterface
{
private:
    boost::shared_ptr<SerialNode> SerialPort;
    boost::thread node_thread;
public:
    SerialInterface();
    virtual ~SerialInterface();
    void StartThread();
    void setGimbal(float yaw, float pitch);
    std::vector<float> readGimbal();
    void setShoot(bool shoot);
};
