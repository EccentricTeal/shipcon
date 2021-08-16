#include "shipcon_drivers/gyro/jae_jg35fd.hh"

namespace shipcon::shipcon_drivers::gyro
{
  DriverNodeJAEJG35FD::DriverNodeJAEJG35FD( std::string node_name, std::string name_space, std::string serial_port_path  ):
  rclcpp::Node( node_name, name_space ),
  serial_socket_( iosrv_, serial_port_path )
  {
    initSerialCom( 9615 );
  }

  DriverNodeJAEJG35FD::~DriverNodeJAEJG35FD()
  {
    ;
  }

  void DriverNodeJAEJG35FD::initSerialCom( unsigned int baudrate_kbps )
  {
    serial_socket_.set_option( boost::asio::serial_port_base::baud_rate( baudrate_kbps ) );
  }

}