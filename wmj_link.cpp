#include "wmj_link.h"
#include <iostream>
#include <string.h>

//analysis a compelete package from the stm32, return 1 else return 0
unsigned char WMJLink::byteAnalysisCall(const unsigned char rx_byte)
{
    unsigned char package_update=0;
    unsigned char receive_message_update=0;
    receive_message_update=receiveFiniteStates(rx_byte) ;  //statemachine analysis, if analysis succeeded, return true
    if( receive_message_update ==1 )
    {
        receive_message_update = 0;
        package_update=packageAnalysis();//analysis the command_id and do the communication work
        if(package_update==1) receive_package_count++;
        return package_update;
    }
    return 0;
}

//statemachine analysis function
unsigned char WMJLink::putErrorInfo(unsigned char error_code)
{
    std::cout << "Sending Error Code" << std::endl;
    switch (error_code)
    {
        case 1:
            std::cout << "Slave Computer Check Sum Error" << std::endl;
            break;

        case 2:
            std::cout << "Slave Computer Cannot Control Gimbal" << std::endl;
            break;

        case 4:
            std::cout << "Slave Computer Didn't Receive Feedback from Servo Machine" << std::endl;
            break;

        case 8:
            std::cout << "Unvalid Package Format" << std::endl;
            break;

        case 16:
            std::cout << "Pitch Reach the Limit" << std::endl;
            break;

        case 32:
            std::cout << "Yaw Reach the Limit" << std::endl;
            break;

        default:
            std::cout << "Unknow Error" << std::endl;
    }
    return 1;
}

unsigned char WMJLink::receiveFiniteStates(const unsigned char rx_data)
{

    switch (receive_state)
    {
    case WAITING_FF1:
        if (rx_data == 0xff)
        {
            // std::cout << "Reading FF1" << std::endl;
            receive_state = WAITING_FF2;
            receive_check_sum =0;
            byte_count=0;
        }
        break;

    case WAITING_FF2:
        // std::cout << "Reading FF2" << std::endl;
        if (rx_data == 0xff)        
            receive_state = FRAPS_NUMBER;
        else
            receive_state = WAITING_FF1;
        break;
    case FRAPS_NUMBER:
        // std::cout << "Reading FRAPS_NUMBER" << std::endl;
        // std::cout << "Fraps Number is" << (int)rx_data << std::endl;
        receive_check_sum = (receive_check_sum+rx_data)%255;
        receive_state = COMMAND_ID;
        break;

    case COMMAND_ID:
        //judge whether the command_id beyond the range of the avaiable command_ids
        //please modify this range if extending the instruction set
        // std::cout << "Reading COMMAND_ID" << std::endl;
        // std::cout << (int)rx_data << std::endl;
        if (rx_data>0x03){
            // std::cerr<<"command_id beyond range error"<<std::endl;
            receive_state = WAITING_FF1;
        }
        else{
            receive_check_sum = (receive_check_sum+rx_data)%255;
            rx_message.command_id = rx_data;
            receive_state = RECEIVE_LEN;
        }
        break;

    case RECEIVE_LEN:
        // std::cout << "Reading LEN" << std::endl;
        receive_check_sum = (receive_check_sum+rx_data)%255;
        receive_message_length = rx_data;
        rx_message.length = receive_message_length;
        // std::cout << rx_message.length << "\n\n\n\n\n\n\n\n";
        receive_state = RECEIVE_PACKAGE;
        break;

    case RECEIVE_PACKAGE:
        // std::cout << "Reading PACKAGE" << std::endl;
        // std::cout << "data is " << (int)rx_data << std::endl;
        receive_check_sum = (receive_check_sum+rx_data)%255;
        rx_message.data[byte_count++] = rx_data;
        if(byte_count >= receive_message_length-6)
        {
            // std::cout << "PACKAGE RECEIVED" << std::endl;
            receive_state = RECEIVE_CHECK;
        }
        break;

    case RECEIVE_CHECK:
        // std::cout << "Reading CHECKSUM" << std::endl;
        // std::cout << (int)rx_data << std::endl;
        if(rx_data == (unsigned char)receive_check_sum)
        {
            // std::cout << "Check Sum = " << receive_check_sum << std::endl;
            receive_check_sum=0;
            receive_state = WAITING_FF1;
            // std::cerr << "receive a package \n";
            return 1;
        }
        else
        {
            // std::cout << "Check Sum = " << receive_check_sum << std::endl;
            std::cerr << "check sum error \n";
            receive_state = WAITING_FF1;
        }
        break;
    default:
        receive_state = WAITING_FF1;
    }
    return 0;
}

//analysis the command_id and do communicate work
unsigned char WMJLink::packageAnalysis(void)
{

    unsigned char analysis_state=0;
    void* single_command;

    command_state = (Command)rx_message.command_id;

   
    switch (command_state)
    {
    case ERROR_CODE :
        //for debug use
        analysis_state=putErrorInfo(rx_message.data[0]);
        break;
 
    case SET_CHASSIS :
        analysis_state=setCommandAnalysis(command_state , 0);
        break;

    case SET_GIMBAL :
        analysis_state=setCommandAnalysis(command_state , sizeof(my_robot->expect_gimbal_pose));
        break;
 
    case READ_GIMBAL :
        analysis_state=readCommandAnalysis(command_state , (unsigned char *)&my_robot->measure_gimbal_pose , sizeof(my_robot->measure_gimbal_pose));
        break;

    case SHOOT :
        analysis_state=readCommandAnalysis(command_state , (unsigned char *)&my_robot->is_shooting , sizeof(my_robot->is_shooting));
        break;
    default :
        analysis_state=0;
        break; 
    }
    rx_message.length=0;
    rx_message.command_id=0;
    return analysis_state;
}

//deal the read command
//simple copy the rx_message.data to the measure_* struct of robot_abstract
unsigned char WMJLink::readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len)
{    
     // master   , means the slave feedback a package to master , and the master save this package
        if((rx_message.length-6) != len)
        {
            std::cerr<<"can not read the message from slave, the length is not mathcing to my expect struct \n";
            return 0;
        }
        memcpy(p, rx_message.data, len);
        receive_package_renew[(unsigned char)command_state] = 1 ;
    return 1;
}

//deal set command 
unsigned char WMJLink::setCommandAnalysis(const Command command_state , const unsigned short int len)
{
    //show the master receive the set command feedback from the slave
    if((rx_message.length-6) != len)
    {
        //there are many error...
        // std::cerr<<"slave is sending an wrong feedback \n";
        return 0;
    }
        // std::cout<<"received a ack~";
    receive_package_renew[(unsigned char)command_state] = 1 ;
    return 1;
}

//use for master to send message
unsigned char WMJLink::masterSendCommand(const Command command_state,const int cnt)
{
    unsigned char analysis_state =1;
    void* single_command;
    
    switch (command_state)
    {
    case ERROR_CODE :
        //for debug use
        // std::cout<<"oniqyan heki?"<<std::endl;
        break;
 
    case SET_GIMBAL :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->expect_gimbal_pose , sizeof(my_robot->expect_gimbal_pose), cnt);
        // std::cout << sizeof(my_robot->expect_gimbal_pose) << std::endl;
        break;
 
    case READ_GIMBAL :
        receive_package_renew[(unsigned char)command_state] = 0 ;    
        sendStruct(command_state , (unsigned char *)single_command , 1, cnt);
        break;

    case SHOOT :
        sendStruct(command_state , (unsigned char *)&my_robot->is_shooting , sizeof(my_robot->is_shooting), cnt);
        break;
    default :
        analysis_state=0;
        break; 
    }
    return analysis_state;
}

//WMJLink::sendStruct implement the tx_message content generated
void WMJLink::sendStruct(const Command command_state , unsigned char* p , const unsigned short int len, unsigned char fraps)
{
    tx_message.length=len;
    tx_message.fraps_number = fraps;
    tx_message.command_id = (unsigned char)command_state;
    if (len == 1)
    {
        //len == 0 mean we will send a read command
        tx_message.data[0] = 0;
    }
    else if(len > 1)
    {
        // memcpy(&tx_message.data , p , len);
        for(int i = 0; i < 4; i++)
        {
            tx_message.data[i] = p[3-i];
            tx_message.data[4+i] = p[7-i];
        }
    }
    sendMessage();
}

//transform tx_message to hex sequence
void WMJLink::sendMessage(void)
{
    unsigned short int tx_i;
    unsigned int check_sum_=0;

    tx_buffer[0] = 0xff;

    tx_buffer[1] = 0xff;

    tx_buffer[2] = tx_message.fraps_number;
    check_sum_ += tx_buffer[2];


    tx_buffer[3] = (unsigned char)( tx_message.command_id); //command_id
    check_sum_ += tx_buffer[3];

    tx_buffer[4] = (unsigned char)tx_message.length+6;   //LEN
    check_sum_ += tx_buffer[4];

    for(tx_i = 0; tx_i < tx_message.length ; tx_i++)   //package
    {
        tx_buffer[5+tx_i] = tx_message.data[tx_i];
        check_sum_ += tx_buffer[5+tx_i];
    }
    check_sum_=check_sum_%255;
    tx_buffer[5+tx_i] = check_sum_;

    tx_buffer_length = 6 + tx_i;

    send_packag_count++;
}
//end 