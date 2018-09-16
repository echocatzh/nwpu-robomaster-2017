//此部分代码可以参照boost库完全开发指南12.2.7 异步socket处理
//以及博客https://www.cnblogs.com/TianFang/archive/2013/02/02/2890366.html
#include "transport_serial.h"

TransportSerial::TransportSerial() :
    Transport("serial:///dev/ttyUSB0")//使用参数初始化表对基类进行构造
{
    params_.serialPort = "/dev/ttyUSB0";
    if (!initializeSerial())
    {
        std::cerr << "serial Transport initialize failed ,please check your system" <<std::endl;
        initialize_ok_ = false;
    } else
    {
        std::cout << "transport initialize ready" <<std::endl;
        initialize_ok_ = true;
    }
}

TransportSerial::TransportSerial(std::string url) :
    Transport(url)
{
    if (comm_url_.substr(0, comm_url_.find("://")) != "serial")
    {
        std::cerr << "url error, please correct your config" <<std::endl;
        return ;
    }
    params_.serialPort = comm_url_.substr(comm_url_.find("://")+ 3, comm_url_.length() - comm_url_.find("://"));
    if (!initializeSerial())
    {
        std::cerr << "serial Transport initialize failed ,please check your system" <<std::endl;
        initialize_ok_ =  false;
    }
    else
    {
        std::cout << "transport initialize ready" <<std::endl;
        initialize_ok_ =  true;
    }
}

void TransportSerial::mainRun()//重复从底层读取data型数据包
{
    std::cout << "Transport main read/write started" <<std::endl;
    boost::asio::io_service::work work(*ios_);
    start_a_read();
    //如果io事件没有异常退出，则可以看到一直是有io事件任务的，但是如果读操作异常，下句就会退出，所以是否需要守护io_service？
    //因为回调函数是在run函数的线程下执行的，所以尽量控制回调函数短一点，免得阻塞后续io响应。
    ios_->run();
}

void TransportSerial::start_a_read()
{
    boost::mutex::scoped_lock lock(port_mutex_);
    // std::cout << "Port successfully Locked~" << std::endl;

    port_->async_read_some(boost::asio::buffer(temp_read_buf_),
                           boost::bind(&TransportSerial::readHandler,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred
                                       ));
}

void TransportSerial::readHandler(const boost::system::error_code &ec, size_t bytesTransferred)
{
    if (ec)
    {
        std::cerr << "Transport Serial read Error "<< std::endl;
        return;
    }
    //std::cout << "ReadHandler successfully Called~" << std::endl;
    boost::mutex::scoped_lock lock(read_mutex_);
    // std::cout << "Read buffer successfully Locked~" << std::endl;
    if(!temp_read_buf_.empty())
    {
        Buffer data(temp_read_buf_.begin(), temp_read_buf_.begin() + bytesTransferred);
        read_buffer_.push(data);//read_buffer_用来存储读上来的data包
    }
    start_a_read();
}

void TransportSerial::start_a_write()
{
    boost::mutex::scoped_lock lock(port_mutex_);
    //std::cout << "Port successfully locked when write~" << std::endl;

    if (!write_buffer_.empty())
    {
        boost::asio::async_write(*port_, boost::asio::buffer((write_buffer_.front())),
                                 boost::bind(&TransportSerial::writeHandler, this, boost::asio::placeholders::error));
        //write_buffer_.pop();
    }
}

void TransportSerial::writeHandler(const boost::system::error_code &ec)
{
    if (ec)
    {
        std::cerr << "Transport Serial write Error "<< std::endl;
        return;
    }
    boost::mutex::scoped_lock lock(write_mutex_);
    if(!write_buffer_.empty())
    {
        write_buffer_.pop();
    }
    //boost::mutex::scoped_lock lock(write_mutex_);
 
    // if (!write_buffer_.empty())	
    // {
        // std::cout << "Ready to start a write" << std::endl;
        //start_a_write();
    // }
}

Buffer TransportSerial::readBuffer()
{
    boost::mutex::scoped_lock lock(read_mutex_);
    //std::cout << "Read BUFFER successfully locked ~" << std::endl;
    // start_a_read();
    if (!read_buffer_.empty())//is not empty
    {
        Buffer data(read_buffer_.front());
        read_buffer_.pop();
        return data;
    }
    //else return empty data
    Buffer data;
    return data;
}

void TransportSerial::writeBuffer(Buffer &data)
{
    boost::mutex::scoped_lock lock(write_mutex_);
    // std::cout << "Write Buffer Successfully Locked~" << std::endl;

    write_buffer_.push(data);
    start_a_write();
}

bool TransportSerial::initializeSerial()
{
    try//初始化串口的传输参数
    {
        std::cout<<params_.serialPort <<std::endl;
        //params for structing serial_port object:
        //io_serivice object and the name of the serialport(in string format)
        port_ = boost::make_shared<boost::asio::serial_port>(boost::ref(*ios_), params_.serialPort);
        // Set an option on the serial port.
        port_->set_option(
                    boost::asio::serial_port::baud_rate(params_.baudRate));
        port_->set_option(
                    boost::asio::serial_port::flow_control((boost::asio::serial_port::flow_control::type)params_.flowControl));
        port_->set_option(
                    boost::asio::serial_port::parity((boost::asio::serial_port::parity::type)params_.parity));
        port_->set_option(
                    boost::asio::serial_port::stop_bits((boost::asio::serial_port::stop_bits::type)params_.stopBits));
        port_->set_option(boost::asio::serial_port::character_size(8));

    }
    catch(std::exception &e)
    {
        std::cerr << "Failed to open the serial port " << std::endl;
        std::cerr << "Error info is "<< e.what() << std::endl;
        return false;
    }

    temp_read_buf_.resize(1024, 0);
    try
    {
        thread_ = boost::thread(boost::bind(&TransportSerial::mainRun, this));//创建新线程执行mainrun()函数->主要用来处理io事件的回调函数
    }
    catch(std::exception &e)
    {
        std::cerr << "Transport Serial thread create failed " << std::endl;
        std::cerr << "Error Info: " << e.what() <<std::endl;
        return false;
    }
    thread_.detach();

    return true;
}


