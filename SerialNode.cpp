#include "SerialNode.h"

SerialNode::SerialNode(std::string url)
{
    std::string transport_method = url.substr(0, url.find("://"));
    if (transport_method == "serial")
    {
        port = boost::make_shared<TransportSerial>(url);
        while(!port->initialize_ok());
        time_out = 20;
        wmjlink = boost::make_shared<WMJLink>(&my_robot);
    }
    //extend for communicate method?
    else if (transport_method == "udp")
    {
    }
    else if (transport_method == "tcp")
    {
    }

    //process the config filr
    file.open("../SerialConfig.cfg", std::fstream::in);
    if (file.is_open())
    {
        for (int i = 0; i < LAST_COMMAND_FLAG; i++)
        {
            std::string temp;
            file >> temp >> wmjlink_command_set[i] >> wmjlink_freq[i];
            // std::cout<< temp << wmjlink_command_set[i] << wmjlink_freq[i]<<std::endl;
        }
        file.close();
        initialize_ok = port->initialize_ok();
    } 
    else
    {
        std::cerr << "config file can't be opened, check your file load " <<std::endl;
        initialize_ok = false;
    }
}


bool SerialNode::updateCommand(const Command &command, int count)
{
    boost::asio::deadline_timer cicle_timer_(*(port->getIOinstace()));
    cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out));//值过低会timeout
    // update command set  data from embedded system
    // std::cout << "prepared to set command~" << std::endl;
    int cnt = count % 100;
    if (wmjlink_command_set[command] != 0)
    {
            // std::cout << "Prepared to send command~~~~" << std::endl;
            sendCommand(command,cnt);//根据command的类型发送数据，已写入串口
            //return true;
    }
    else return false;
    //上面的代码完成定时向stm32发送指令，下面的代码完成分析接收包的信息
    // 相当于每发送一次指令后,程序就卡在下述代码段等待数据反馈
    Buffer data = port->readBuffer();
    while(!data.size())
    {
        data = port->readBuffer();
    }
    // std::cout << "Readbuffer successfully executed~" << std::endl;
    ack_ready = false;
    // boost::mutex::scoped_lock lock(command_set_mutex);
    while (!ack_ready)
    {
        for (int i = 0; i < data.size(); i++)
        {
            // std::cout << "data" << i << "=" << (int)data[i] << std::endl;
            if (wmjlink->byteAnalysisCall(data[i]))//分析包的完整性，完整时分析包并执行所属操作,对机器人ADT的变量更新即在这层调用完成
            {
                // one package ack arrived
                ack_ready = true;
            }
            else ack_ready = false;
        }
        if(ack_ready)
        {
            return true;
        }
        else
        {
            //std::cerr << "FeedBack error!!!!" << std::endl;
            return updateCommand(command,cnt);
        }
        //data = port->readBuffer();//取出早先从底层读出存储到read_buffer_的包
        if (cicle_timer_.expires_from_now().is_negative())//超时报错
        {
            std::cerr<<"Timeout continue skip this package "<<command<<std::endl;
            return false;
        }
    }
}

void SerialNode::mainRun()
{
    unsigned int time_delay = 1000/wmjlink_freq[1];
    unsigned int count = 0;
    // serial thread work
    // std::cout << "Prepared to update robot_abstract~" << std::endl;
    while(1){
        updateCommand(SET_GIMBAL, count++);
        updateCommand(READ_GIMBAL, count++);
        Delay(time_delay);
    }
}
