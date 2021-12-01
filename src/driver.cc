#include "shipcondev_gyro_jg35fd/driver.hh"

namespace shipcon::device
{
  GyroJaeJG35FD::GyroJaeJG35FD( std::string node_name, std::string name_space ):
  rclcpp::Node( node_name, name_space )
  {
    //init ROS Service
    srvname_control_output_ = declare_parameter( "servicename/control_output", "control_output" );
    srv_control_output_ = create_service<shipcondev_gyro_jg35fd::srv::ControlOutput>(
      srvname_control_output_,
      &GyroJaeJG35FD::callback_srv_control_output
    );
    srvname_calibrate_bias_drift_ = declare_parameter( "servicename/calibrate_bias_drift", "calibrate_bias_drift" );
    srv_calibrate_bias_drift_ = create_service<shipcondev_gyro_jg35fd::srv::CalibrateBiasDrift>(
      srvname_calibrate_bias_drift_,
      &GyroJaeJG35FD::callback_srv_calibrate_bias_drift
    );
    srvname_control_calculate_ = declare_parameter( "servicename/control_calculate", "control_calculate" );
    srv_control_calculate_ = create_service<shipcondev_gyro_jg35fd::srv::ControlCalculate>(
      srvname_control_calculate_,
      &GyroJaeJG35FD::callback_srv_control_calculate
    );
    srvname_reset_angle_ = declare_parameter( "servicename/reset_angle", "reset_angle" );
    srv_reset_angle_ = create_service<shipcondev_gyro_jg35fd::srv::ResetAngle>(
      srvname_reset_angle_,
      &GyroJaeJG35FD::callback_srv_reset_angle
    );
    srvname_set_analog_range_ = declare_parameter( "servicename/set_analog_range", "set_analog_rage" );
    srv_set_analog_range_ = create_service<shipcondev_gyro_jg35fd::srv::SetAnalogRange>(
      srvname_set_analog_range_,
      &GyroJaeJG35FD::callback_srv_set_analog_range
    );

    //init serial device
    if ( initSerial() ){ serialif_->run(); }

    //init data variables
    data_buffer_.clear();
    yaw_angle_ = 0.0;

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
    //std::cout << std::dec << sendsize << std::endl;
  }


  void GyroJaeJG35FD::callback_receive_header( const boost::system::error_code& ec, std::size_t recvsize )
  {
    std::istream istr( &recv_buffer_ );

    //Set header data(1 or 2 bytes) from istream 
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
      data_buffer_.push_back( *( temp.end() - 3 ) ); //0x02
      data_buffer_.push_back( *( temp.end() - 2 ) ); //0x81-0x84
      //Last istream byte( temp.end()-1 ) is 0xff
    }
    
    //Receiving remaining data packet
    //0x81, 0x82 or 0x84 message contains 2byte data
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
    //0x83 message contains 4byte data
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
    //If next 0x02 data is not 0x81-0x84, this 0x02 is not indicator of STX of header
    //So, read until header again. This case is only in an error.
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
    std::istream istr( &recv_buffer_ );
        
    //This callback functions receive only 5 or 7 bytes normally.
    //In case receiving the other number of data, it is error, so going back to header receive process
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

    //receive data (discard final 0xff of istream)
    for( int cnt = 2; cnt < ( 2 + recvsize ); cnt++ ){ data_buffer_.push_back( istr.get() ); }
    recv_buffer_.consume( recv_buffer_.size() );

    //Evaluate final byte whether 0x0d or not
    //In case the header got previous process is true header, update data variable.
    if( *( std::prev( data_buffer_.end() ) ) == 0x0d )//in case received header is true header
    {
      
      updateData();
    }
    //In case received header is just a part of data
    else
    {
      //Find next header 0x02 + 0x81-0x84
      auto itr = data_buffer_.begin()+2; //From next "not true header"
      for( ; itr != data_buffer_.end(); ++itr )
      {
        itr = std::find( itr, data_buffer_.end(), 0x02 );

        //Not found
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
        //Found 0x02 in the end of buffer
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
        //Found 0x02 not in the end of buffer
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
    uint16_t angle = 0;
    int16_t rate = 0;

    if( data_buffer_[1] == 0x81 )
    {
      uint8_t checksum = static_cast<uint8_t>( data_buffer_[1] + data_buffer_[2] + data_buffer_[3] + data_buffer_[4]);
      if( checksum == data_buffer_[5] )
      {
        angle = static_cast<uint16_t>( data_buffer_[3] << 8 ) + data_buffer_[4];

        std::lock_guard<std::mutex> lock(mtx_);
        yaw_angle_ = static_cast<double>( angle ) / static_cast<double>( 0xffff ) * M_PI * 2;
      }
    }
    else if( data_buffer_[1] == 0x82)
    {
      uint8_t checksum = static_cast<uint8_t>( data_buffer_[1] + data_buffer_[2] + data_buffer_[3] + data_buffer_[4]);
      if( checksum == data_buffer_[5] )
      {
        rate = static_cast<int16_t>( data_buffer_[3] << 8 ) + data_buffer_[4];

        std::lock_guard<std::mutex> lock(mtx_);
        yaw_rate_ = static_cast<double>( rate ) / static_cast<double>( 0x7fff ) * deg2rad( 200.0 );
      }
    }
    else if( data_buffer_[1] == 0x83)
    {
      uint8_t checksum = static_cast<uint8_t>( data_buffer_[1] + data_buffer_[2] + data_buffer_[3] + data_buffer_[4] + data_buffer_[5] + data_buffer_[6]);
      if( checksum == data_buffer_[7] )
      {
        angle = static_cast<uint16_t>( data_buffer_[3] << 8 ) + data_buffer_[4];
        rate = static_cast<int16_t>( data_buffer_[5] << 8 ) + data_buffer_[6];

        std::lock_guard<std::mutex> lock(mtx_);
        yaw_angle_ = static_cast<double>( angle ) / static_cast<double>( 0xffff ) * M_PI * 2.0 ;
        yaw_rate_ = static_cast<double>( rate ) / static_cast<double>( 0x7fff ) * deg2rad( 200.0 );
      }
    }
  
    data_buffer_.clear();

    std::cout << "Yaw angle is " << std::fixed << std::setprecision(6) << yaw_angle_ << " rad" << std::endl;
    std::cout << "Yaw rate is " << std::fixed << std::setprecision(6) << yaw_rate_ << " rad/s" << std::endl;

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


  void GyroJaeJG35FD::configureOutput( TxInterval interval, OutputMode mode )
  {
    uint8_t checksum = static_cast<uint8_t>( mode + interval );

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( mode );
    send_buffer_->push_back( interval );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJaeJG35FD::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );    
  }


  void GyroJaeJG35FD::resetAngle( double new_angle )
  {
    int16_t angle = static_cast<int16_t>( new_angle * 32767.0 / 180.0 );
    uint8_t checksum = static_cast<uint8_t>( 0x85 + interval );

    auto send_buffer_ = std::make_shared<std::vector<unsigned char>>();
    send_buffer_->push_back( 0x02 );
    send_buffer_->push_back( 0x85 );
    send_buffer_->push_back( interval );
    send_buffer_->push_back( checksum );
    send_buffer_->push_back( 0x0d );

    serialif_->dispatchSend(
      send_buffer_,
      std::bind(
        &GyroJaeJG35FD::callback_sendSerial,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );    
  }


  void GyroJaeJG35FD::callback_srv_control_output(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlOutput_Request> req,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlOutput_Response> res
  )
  {

  }


  void GyroJaeJG35FD::callback_srv_calibrate_bias_drift(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::CalibrateBiasDrift_Request> req,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::CalibrateBiasDrift_Response> res
  )
  {

  }


  void GyroJaeJG35FD::callback_srv_control_calculate(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlCalculate_Request> req,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlCalculate_Response> res
  )
  {

  }


  void GyroJaeJG35FD::callback_srv_reset_angle(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ResetAngle_Request> req,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ResetAngle_Response> res
  )
  {

  }


  void GyroJaeJG35FD::callback_srv_set_analog_range(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::SetAnalogRange_Request> req,
    const std::shared_ptr<shipcondev_gyro_jg35fd::srv::SetAnalogRange_Response> res
  )
  {
    
  }


}