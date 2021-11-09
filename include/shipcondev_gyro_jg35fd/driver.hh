#ifndef SHIPCONDEV_GYRO_JG35FD__DRIVER__HH
#define SHIPCONDEV_GYRO_JG35FD__DRIVER__HH

//ROS Packages
#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/serialcom.hh"

//STL
#include <memory>
#include <string>
#include <iostream>
#include <boost/asio.hpp>

namespace shipcon::device
{
  class GyroJaeJG35FD : public rclcpp::Node
  {
    /* Constants */
    private:
      const int BAUDRATE = 9600;
      enum TxInterval{
        once = 0x30,
        _20ms = 0x32,
        _50ms = 0x33,
        _100ms = 0x34,
        _200ms = 0x35,
        _250ms = 0x36,
        _500ms = 0x37,
        _1000ms = 0x38,
        stop = 0x39
      };

    /* Constructor, Destructor */
    public:
      GyroJaeJG35FD( std::string node_name, std::string name_space );
      ~GyroJaeJG35FD();

    /* Private Methods */
    private:
      bool initSerial( void );
      bool startSerial( void );
      void callback_sendSerial( const boost::system::error_code& ec, std::size_t sendsize );
      void callback_receiveSerial( const boost::system::error_code& ec, std::size_t recvsize );

    /* Private Member Objects*/
    private:
      std::unique_ptr<hwcomlib::SerialCom> serialif_;
      boost::asio::streambuf recv_buffer_;
  };
}

#endif