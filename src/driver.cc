#include "shipcondev_gyro_jg35fd/driver.hh"

namespace shipcon::device
{
  GyroJaeJG35FD::GyroJaeJG35FD( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    if ( initSerial() ){ serialif_->run(); }

    std::vector<unsigned char> buf;
    buf.push_back(0x02);
    buf.push_back(0x81);
    buf.push_back(0x35);
    buf.push_back(0xb6);
    buf.push_back(0x0d);
    serialif_->dispatchSend(
      buf,
      std::bind(
        &GyroJaeJG35FD::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    //recv_buffer_.clear();
    serialif_->dispatchRecvUntil(
      recv_buffer_,
      "\r",
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
    std::cout << "Received:" << std::dec << recvsize << std::endl;
    std::istream istr( &recv_buffer_ );
    std::vector<unsigned char> value;

    while( istr )
    {
      value.push_back( istr.get() );
      if( *( value.end()-1 ) == 0x0d ){ break; }
    }
    for( auto itr = value.begin(); itr!=value.end(); ++itr)
    {
      std::cout << "0x" << std::hex << static_cast<int>(*itr) << " ";
    }
    std::cout << std::endl;
    
    serialif_->dispatchRecvUntil(
      recv_buffer_,
      REGEX_CONDITION,
      std::bind(
        &GyroJaeJG35FD::callback_receiveSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }


}