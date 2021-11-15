#include "shipcondev_gyro_jg35fd/driver.hh"

namespace shipcon::device
{
  GyroJaeJG35FD::GyroJaeJG35FD( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    if ( initSerial() ){ serialif_->run(); }
    data_buffer_.clear();
    yaw_angle_ = 0.0;

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
      REGEX_CONDITION_HEADER,
      std::bind(
        &GyroJaeJG35FD::callback_receive_header,
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


  void GyroJaeJG35FD::callback_receive_header( const boost::system::error_code& ec, std::size_t recvsize )
  {
    std::cout << "Received Header:" << std::dec << recvsize << std::endl;
    std::istream istr( &recv_buffer_ );

    if( recvsize == 1 )
    {
      data_buffer_.push_back( istr.get() );
    }
    else
    {
      std::vector<unsigned char> temp;
      temp.clear();
      while( istr ){ temp.push_back( istr.get() ); }
      
      data_buffer_.clear();
      //istr.seekg( -1, std::ios_base::end );
      data_buffer_.push_back( *( temp.end() - 3 ) ); //0x02
      data_buffer_.push_back( *( temp.end() - 2 ) ); //0x81-0x84
      //recv_buffer_.consume( recv_buffer_.size() );
    }
    
    std::cout << "0x" << std::hex << static_cast<int>(data_buffer_[0]) << " ";
    std::cout << "0x" << std::hex << static_cast<int>(data_buffer_[1]) << std::endl;

    
    if( data_buffer_[1] == 0x81 || data_buffer_[1] == 0x82 || data_buffer_[1] == 0x84 )
    {
      serialif_->dispatchRecvSize(
        recv_buffer_,
        5,
        std::bind(
          &GyroJaeJG35FD::callback_receive_data,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          2
        )
      );
    }
    else if( data_buffer_[1] == 0x83 )
    {
      serialif_->dispatchRecvSize(
        recv_buffer_,
        7,
        std::bind(
          &GyroJaeJG35FD::callback_receive_data,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          4
        )
      );
    }
    else
    {
      serialif_->dispatchRecvUntil(
      recv_buffer_,
      REGEX_CONDITION_HEADER,
      std::bind(
        &GyroJaeJG35FD::callback_receive_header,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
    }
  }

  void GyroJaeJG35FD::callback_receive_data( const boost::system::error_code& ec, std::size_t recvsize, unsigned int datasize )
  {
    std::cout << "Received Data:" << std::dec << recvsize << std::endl;
    std::istream istr( &recv_buffer_ );

    if( recvsize != datasize + 3 )
    {
      serialif_->dispatchRecvUntil(
        recv_buffer_,
        REGEX_CONDITION_HEADER,
        std::bind(
          &GyroJaeJG35FD::callback_receive_header,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );
      return;
    }


    for( int cnt = 2; cnt < ( 2 + recvsize ); cnt++ ){ data_buffer_.push_back( istr.get() ); }
    recv_buffer_.consume( recv_buffer_.size() );

    for( auto itr = data_buffer_.begin(); itr!=data_buffer_.end(); ++itr )
    {
      int data = static_cast<int>(*itr);
      std::cout << "0x" << std::hex << data << " ";
    }
    std::cout << std::endl;


    if( *( std::prev( data_buffer_.end() ) ) == 0x0d )//in case received header is true header
    {
      updateData();
    }
    else //in case received header is just a part of data
    {
      //Find next header 0x02 + 0x81-0x84
      auto itr = data_buffer_.begin()+2; //From next "not true header"
      for( ; itr != data_buffer_.end(); ++itr )
      {
        itr = std::find( itr, data_buffer_.end(), 0x02 );

        if( itr == data_buffer_.end() )
        {
          serialif_->dispatchRecvUntil(
            recv_buffer_,
            REGEX_CONDITION_HEADER,
            std::bind(
              &GyroJaeJG35FD::callback_receive_header,
              this,
              std::placeholders::_1,
              std::placeholders::_2
            )
          );
          break;
        }
        else if( itr == data_buffer_.end() - 1 )
        {
          data_buffer_.clear();
          data_buffer_.push_back( *itr );
          serialif_->dispatchRecvSize(
            recv_buffer_,
            1,
            std::bind(
              &GyroJaeJG35FD::callback_receive_header,
              this,
              std::placeholders::_1,
              std::placeholders::_2
            )
          );
          break;
        }
        else
        {
          if( *( std::next( itr ) ) == 0x81 || *( std::next( itr ) ) == 0x82 || *( std::next( itr ) ) == 0x84 )
          {
            std::vector<unsigned char> tempv;
            tempv.clear();
            for( ; itr != data_buffer_.end(); ++itr )
            {
              tempv.push_back( *itr );
            }
            data_buffer_.clear();
            std::copy( tempv.begin(), tempv.end(), data_buffer_.begin() );

            serialif_->dispatchRecvSize(
              recv_buffer_,
              ( 7 - data_buffer_.size() ),
              std::bind(
                &GyroJaeJG35FD::callback_receive_data,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                2
              )
            );
          }
          else if( *( std::next( itr ) ) == 0x83 )
          {
            std::vector<unsigned char> tempv;
            tempv.clear();
            for( ; itr != data_buffer_.end(); ++itr )
            {
              tempv.push_back( *itr );
            }
            data_buffer_.clear();
            std::copy( tempv.begin(), tempv.end(), data_buffer_.begin() );

            serialif_->dispatchRecvSize(
              recv_buffer_,
              ( 9 - data_buffer_.size() ),
              std::bind(
                &GyroJaeJG35FD::callback_receive_data,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                4
              )
            );
          }
          else{;}
          break;
        }
      }

    }

  }


  void GyroJaeJG35FD::updateData( void )
  {
    std::cout << "Update" << std::endl;
    for( auto itr = data_buffer_.begin(); itr != data_buffer_.end(); ++itr)
    {
      int data = static_cast<int>( *itr );
      std::cout << "0x" << std::hex << data << " ";
    }
    std::cout << std::endl;
    data_buffer_.clear();

    serialif_->dispatchRecvUntil(
      recv_buffer_,
      REGEX_CONDITION_HEADER,
      std::bind(
        &GyroJaeJG35FD::callback_receive_header,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }


}