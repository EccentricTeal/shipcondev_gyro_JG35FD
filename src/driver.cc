#include "shipcondev_gyro_jg35fd/driver.hh"

namespace shipcon::device
{
  GyroJaeJG35FD::GyroJaeJG35FD( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    if ( initSerial() ){ serialif_->run(); }

    char buf[5];
    buf[0] = 0x02;
    buf[1] = static_cast<char>(0x81);
    buf[2] = 0x32;
    buf[3] = static_cast<char>(0xb3);
    buf[4] = 0x0d;
    serialif_->dispatchSend(
      buf,
      std::bind(
        &GyroJaeJG35FD::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    serialif_->dispatchRecv(
      buffer_,
      std::bind(
        &GyroJaeJG35FD::callback_receiveSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }

  GyroJaeJG35FD::~GyroJaeJG35FD()
  {
    ;
  }


  bool GyroJaeJG35FD::initSerial( void )
  {
    
    serialif_ = std::make_unique<hwcomlib::SerialCom>( "/dev/ttyUSB0", BAUDRATE );

    if( serialif_ )
    {
      serialif_->setCharacterSize( 8 );
      serialif_->setFlowControl( boost::asio::serial_port_base::flow_control::none );
      serialif_->setParity( boost::asio::serial_port_base::parity::odd );
      serialif_->setStopBits( boost::asio::serial_port_base::stop_bits::one );

      return true;
    }

    return false;
  }


  void GyroJaeJG35FD::callback_sendSerial( const boost::system::error_code& ec, std::size_t sendsize )
  {
    std::cout << std::dec << sendsize << std::endl;
  }


  void GyroJaeJG35FD::callback_receiveSerial( const boost::system::error_code& ec, std::size_t recvsize )
  {
    std::cout << std::dec << recvsize << std::endl;
    std::cout << std::hex << "0x" << buffer_[0] << " 0x" << buffer_[1] << " 0x" << buffer_[2] << " 0x" << buffer_[3] << std::endl; 
  }


}