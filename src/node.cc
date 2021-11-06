#include "shipcondev_gyro_jg35fd/driver.hh"
#include "rclcpp/rclcpp.hpp"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto shipsim_node = std::make_shared<shipcon::device::GyroJaeJG35FD>( "ShipconDevices", "/gyro_jg35fd" );
  rclcpp::spin( shipsim_node );
  rclcpp::shutdown();
}