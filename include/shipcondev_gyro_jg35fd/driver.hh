#ifndef SHIPCONDEV_GYRO_JG35FD__DRIVER__HH
#define SHIPCONDEV_GYRO_JG35FD__DRIVER__HH

//ROS Packages
#include <rclcpp/rclcpp.hpp>
#include "hardware_communication_lib/serialcom.hh"

//ROS Service
#include "shipcondev_gyro_jg35fd/srv/control_output.hpp"
#include "shipcondev_gyro_jg35fd/srv/calibrate_bias_drift.hpp"
#include "shipcondev_gyro_jg35fd/srv/control_calculate.hpp"
#include "shipcondev_gyro_jg35fd/srv/reset_angle.hpp"
#include "shipcondev_gyro_jg35fd/srv/set_analog_range.hpp"

//STL
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <vector>
#include <cmath>
#include <utility>
#include <boost/asio.hpp>
#include <boost/regex.hpp>

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
      enum OutputMode{
        yaw_angle = 0x81,
        yaw_rate = 0x82,
        both = 0x83
      };
      enum AnalogAngleRange{
        current = 0x30,
        _10deg = 0x31,
        _20deg = 0x32,
        _45deg = 0x33,
        _90deg = 0x34,
        _180deg = 0x35
      };
      enum AnalogYawrateRange{
        current = 0x30,
        _10deg = 0x31,
        _20deg = 0x32,
        _50deg = 0x33,
        _90deg = 0x34,
        _200deg = 0x35
      };
      boost::regex REGEX_CONDITION_HEADER = boost::regex("\x02[\x81-\x84]");

    /* Constructor, Destructor */
    public:
      GyroJaeJG35FD( std::string node_name, std::string name_space );
      ~GyroJaeJG35FD();

    /* Private Methods */
    private:
      //Serial Communication
      bool initSerial( void );
      bool startSerial( void );
      void callback_sendSerial( const boost::system::error_code& ec, std::size_t sendsize );
      void callback_receive_header( const boost::system::error_code& ec, std::size_t recvsize );
      void callback_receive_data( const boost::system::error_code& ec, std::size_t recvsize, unsigned int datasize );
      void updateData( void );
      double deg2rad( double deg ){ return deg / 360.0 * M_PI * 2.0; };

      //Gyro Applications
      void configureOutput( TxInterval interval, OutputMode mode );
      void resetAngle( double new_angle );
      bool configureInternalCalculator( bool isEnable );
      AnalogAngleRange setAnalogAngleRange( AnalogAngleRange target );
      AnalogYawrateRange setAnalogYawrateRange( AnalogYawrateRange target );

      //ROS Service
      void callback_srv_control_output(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlOutput_Request> req,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlOutput_Response> res
      );
      void callback_srv_calibrate_bias_drift(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::CalibrateBiasDrift_Request> req,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::CalibrateBiasDrift_Response> res
      );
      void callback_srv_control_calculate(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlCalculate_Request> req,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ControlCalculate_Response> res
      );
      void callback_srv_reset_angle(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ResetAngle_Request> req,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::ResetAngle_Response> res
      );
      void callback_srv_set_analog_range(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::SetAnalogRange_Request> req,
        const std::shared_ptr<shipcondev_gyro_jg35fd::srv::SetAnalogRange_Response> res
      );

    /* Private Member Objects*/
    private:
      //Serial Communication
      std::unique_ptr<hwcomlib::SerialCom> serialif_;
      
      //ROS Service
      rclcpp::Service<shipcondev_gyro_jg35fd::srv::ControlOutput>::SharedPtr srv_control_output_;
      rclcpp::Service<shipcondev_gyro_jg35fd::srv::CalibrateBiasDrift>::SharedPtr srv_calibrate_bias_drift_;
      rclcpp::Service<shipcondev_gyro_jg35fd::srv::ControlCalculate>::SharedPtr srv_control_calculate_;
      rclcpp::Service<shipcondev_gyro_jg35fd::srv::ResetAngle>::SharedPtr srv_reset_angle_;
      rclcpp::Service<shipcondev_gyro_jg35fd::srv::SetAnalogRange>::SharedPtr srv_set_analog_range_;
      std::string srvname_control_output_;
      std::string srvname_calibrate_bias_drift_;
      std::string srvname_control_calculate_;
      std::string srvname_reset_angle_;
      std::string srvname_set_analog_range_;
      
      //Buffers
      boost::asio::streambuf recv_buffer_;
      std::vector<unsigned char> data_buffer_;
      //Data
      double yaw_angle_;
      double yaw_rate_;
      //Utility
      std::mutex mtx_;
  };
}

#endif