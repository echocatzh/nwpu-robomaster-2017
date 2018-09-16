#ifndef SERIALNODE_H_
#define SERIALNODE_H_
#include <fstream>
#include "transport_serial.h"
#include "wmj_link.h"
#include <cstdlib>
#include <vector>
#include <ctime>

void Delay(unsigned int millisec)
{
    clock_t start;
    start = clock();
    clock_t microsec = 1000 * millisec;
    while(clock() - start < microsec);
}

class SerialNode{

public:
    SerialNode(std::string url);
    //interface for writing the user data of robot
    void setGimbal(float yaw, float pitch){
        my_robot.expect_gimbal_pose.pitch = pitch;
        my_robot.expect_gimbal_pose.yaw = yaw;
    }
    std::vector<float> readGimbal(void){
        std::vector<float> result(2);
		result[0] = my_robot.measure_gimbal_pose.yaw;
		result[1] = my_robot.measure_gimbal_pose.pitch;
        return result;
    }
    void setShoot(bool is_shooting_){
        //usually we just want the barrel to shoot(true) but not to stop(false)
        my_robot.is_shooting = is_shooting_;
    }
	void mainRun();

private:
	boost::shared_ptr<Transport> port;//factory class of serialport function
	boost::shared_ptr<WMJLink> wmjlink ;
	boost::mutex command_set_mutex;

	//for reading config file
	std::fstream file;
	bool initialize_ok;
	//for updating data
	int wmjlink_command_set[LAST_COMMAND_FLAG];
	int wmjlink_count[LAST_COMMAND_FLAG];
	int wmjlink_command_set_current[LAST_COMMAND_FLAG];
	int wmjlink_freq[LAST_COMMAND_FLAG];
	bool ack_ready;
	int time_out;
	inline void sendCommand(const Command command_state,const int cnt)
	{
		wmjlink->masterSendCommand(command_state,cnt);
        //construct the in the method of vector(v.begin(),v.size()+v.begin());
		Buffer data(wmjlink->getSerializedData(), wmjlink->getSerializedLength() + wmjlink->getSerializedData());//包装组装好的tx_buffer
		port->writeBuffer(data);//串口写入组装好的tx_buffer
		//std::cout << "command " << cnt << "has been sent" << std::endl;
	}
	bool updateCommand(const Command &command, int count);
	// a single object for robot
	RobotAbstract my_robot;
};

#endif 
