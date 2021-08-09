#include "hwcom_module/serialcom.hh"
/* https://blog.myon.info/entry/2015/04/19/boost-asio-serial/ */

namespace shipcon::hwcom
{
  SerialTransceiver::SerialTransceiver( std::string port ):
  serial_socket_( iosrv_, port )
  {
    ;
  }

  SerialTransceiver::SerialTransceiver( std::string port, unsigned int baudrate ):
  serial_socket_( iosrv_, port )
  {
    serial_socket_.set_option( boost::asio::serial_port_base::baud_rate( baudrate ) );
  }

  SerialTransceiver::~SerialTransceiver()
  {
    ;
  }

  void SerialTransceiver::setBaudRate( unsigned int baudrate )
  {
    serial_socket_.set_option( boost::asio::serial_port_base::baud_rate( baudrate ) );
  }

  void SerialTransceiver::send( std::string& senddata )
  {
    //Blocking
    //TODO: Async
    boost::asio::write( serial_socket_, boost::asio::buffer( senddata ) );
  }

  void SerialTransceiver::recv( std::string& recvdata )
  {
    //Blocking
    

    boost::asio::spawn(
      serial_socket_,
      [this]( boost::asio::yield_context yield )
      {
        boost::asio::streambuf buffer;
        boost::system::error_code error_code;
        
        const size_t recvsize = boost::asio::async_read( 
        serial_socket_,
        buffer,
        boost::asio::transfer_at_least(1),
        yield[error_code]
      );
      }
    )

    while( !error_code )
    {
      
      

    }
    
    
    

    recvdata = "";

  }


}