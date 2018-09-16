
#ifndef TRANSPORT_SERIAL_H_
#define TRANSPORT_SERIAL_H_

#include "transport.h"



class SerialParams {
public:
	std::string serialPort;
	unsigned int baudRate;
	unsigned int flowControl;
	unsigned int parity;
	unsigned int stopBits;
	SerialParams() :
			serialPort(), baudRate(115200), flowControl(0), parity(0), stopBits(0)
	{
	}
	SerialParams(
			std::string _serialPort,
			unsigned int _baudRate,
			unsigned int _flowControl,
			unsigned int _parity,
			unsigned int _stopBits
			) :
			serialPort(_serialPort),
			baudRate(_baudRate),
			flowControl(_flowControl),
			parity(_parity),
			stopBits(_stopBits)
	{
	}
};

//加上串口参数的串口实现类
class TransportSerial : public Transport {

public:
	//default structure function
	TransportSerial ();
	TransportSerial (std::string url);

	virtual Buffer readBuffer();

	virtual void writeBuffer(Buffer &data);

private:
	//io Object 
	boost::shared_ptr<boost::asio::serial_port> port_;
	SerialParams params_;
	// for async read
	Buffer temp_read_buf_;

	boost::thread thread_;
	// locks
	boost::mutex port_mutex_;
	boost::mutex write_mutex_;
	boost::mutex read_mutex_;

	bool initializeSerial();
	void mainRun();

	void start_a_read();
	void start_a_write();
	void readHandler(const boost::system::error_code &ec, size_t bytesTransferred);
	void writeHandler(const boost::system::error_code &ec);
};


#endif /* TRANSPORT_SERIAL_H_ */
