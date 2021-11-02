#include "shipcondev_gyro_JG35FD/driver.hh"

namespace shipcon::device
{
  GyroJaeJG35FD::GyroJaeJG35FD( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    if ( initSerial() ){ serialif_->run(); }

    std::string buffer;
    serialif_->dispatchRecv( buffer, std::bind( &GyroJaeJG35FD::callback_receiveSerial, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );
  }

  GyroJaeJG35FD::~GyroJaeJG35FD()
  {
    ;
  }


  bool GyroJaeJG35FD::initSerial( void )
  {
    serialif_ = std::make_unique<hwcomlib::SerialCom>( BAUDRATE, "/dev/ttyUSB0" );

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


  void GyroJaeJG35FD::callback_receiveSerial( const boost::system::error_code& ec, std::size_t recvsize, std::string& data )
  {
    const char* buffer = data.c_str();
    std::cout << std::hex << "0x" << buffer[0] << " 0x" << buffer[1] << " 0x" << buffer[2] << " 0x" << buffer[3] << std::endl; 
  }


}