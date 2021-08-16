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

  void SerialTransceiver::recv_until( std::string& recvdata, std::string match_expression, int timeout )
  {
    //Non Blocking
    bool isTimeouted = false;
    boost::asio::streambuf buffer;
    boost::asio::deadline_timer timer( io_context );
    timer.expires_from_now( boost::posix_time::seconds( timeout ) );
    
    timer.async_wait(
      [this, &isTimeouted]( const boost::system::error_code& error_code )
      {
        if( error_code == boost::asio::error::operation_aborted )
        {
          //Timer Canceled
          ;
        }
        else
        {
          //Timer out
          serial_socket_.cancel();
          isTimeouted = true;
        }
      }
    );

    //Receive data
    boost::asio::async_read_until(
      serial_socket_,
      buffer,
      match_expression,
      [&timer]( const boost::system::error_code& error_code )
      {
        timer.cancel();
      }
    );

  }


}