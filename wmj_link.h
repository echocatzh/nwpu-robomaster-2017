#ifndef WMJ_LINK_H
#define WMJ_LINK_H
//类成员变量别加_啦，函数参数加上吧，如果是对成员变量的修改
#include "robot_abstract.h"

static const unsigned short int MESSAGE_BUFER_SIZE = 100;  //limite one message's size

typedef struct WMJMessage{
    unsigned char fraps_number;
    unsigned char command_id;
    unsigned short int length;
    unsigned char data[MESSAGE_BUFER_SIZE];
}WMJMessage;

//communication status
enum Recvstate{
    WAITING_FF1,
    WAITING_FF2,
    FRAPS_NUMBER,
    COMMAND_ID,
    RECEIVE_LEN,
    RECEIVE_PACKAGE,
    RECEIVE_CHECK
};

//comand type
enum Command{
    ERROR_CODE,         //0
    SET_GIMBAL,         //1
    SET_CHASSIS,        //2
    READ_GIMBAL,        //3
    SHOOT,              //4
    LAST_COMMAND_FLAG};//5

//要啥扩展性啊，id去啦去啦
class WMJLink
{
    public:
        WMJLink( RobotAbstract* my_robot_ =  0)
        {   
            my_robot=my_robot_;
            receive_state=WAITING_FF1;
            command_state=ERROR_CODE;
            rx_message.fraps_number = 0;
            rx_message.length = 0;
            rx_message.command_id = ERROR_CODE;
            tx_message.fraps_number = 0;
            tx_message.length = 0;
            tx_message.command_id = ERROR_CODE;
            receive_package_count = 0;
            package_update_frequency = 0;
            send_packag_count = 0;
            tx_buffer[0] = 0;
            tx_buffer_length = 0;
        }
        // communicate interface for master
        unsigned char masterSendCommand(const Command command_state,const int cnt);
        inline unsigned char getReceiveRenewFlag(const Command command_state) const
        {
            return receive_package_renew[command_state];
        }
        inline unsigned char* getSerializedData(void) 
        {
            return tx_buffer;
        }
        inline int getSerializedLength(void)
        {
            return tx_buffer_length;
        }
        //the top analysis interface
        unsigned char byteAnalysisCall(const unsigned char rx_byte);


private:


        // robot abstract pointer to wmjlink
        RobotAbstract* my_robot;
        Recvstate   receive_state;
        Command    command_state;
        WMJMessage rx_message , tx_message;
        // boost::mutex robot_abs_mutex;

        float receive_package_count;
        float package_update_frequency;   // how many message receive in one second
        unsigned char receive_package_renew[LAST_COMMAND_FLAG];

        float send_packag_count;
        unsigned char tx_buffer[MESSAGE_BUFER_SIZE + 20];
        unsigned tx_buffer_length;

        unsigned int receive_check_sum;
        short int receive_message_length;
        short int byte_count;
        unsigned char receiveFiniteStates(const unsigned char rx_data);
        unsigned char packageAnalysis(void);
        unsigned char putErrorInfo(unsigned char);
        unsigned char readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
        unsigned char setCommandAnalysis(const Command command_state , const unsigned short int len);
        void sendStruct(const Command, unsigned char*, const unsigned short int, unsigned char);
        void sendMessage(void);
};
#endif