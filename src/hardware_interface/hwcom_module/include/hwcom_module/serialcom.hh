#ifndef SHIPCON__HARDWARE_INTERFACE__SERIALCOM_HH
#define SHIPCON__HARDWARE_INTERFACE__SERIALCOM_HH

/**
 * @file serialcom.hh
 * @brief Simple class library for Serial communication(Send or Receive) in C++ (Not Thread Safe)
 * @author Suisei WADA
 * @date 25th July, 2021
 * 
 * @details
 * @note
 */

#include <string>
#include <memory>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>

/**
 * Core class for Serial communication.
 */
namespace shipcon::hwcom
{
  class SerialTransceiver
  {       
    /* Constructor, Destructor */
    public:
      SerialTransceiver( std::string port );
      SerialTransceiver( std::string port, unsigned int baudrate );
      ~SerialTransceiver();

    public:
      void setBaudRate( unsigned int baudrate );
      void send( std::string& senddata );
      void recv_until( std::string& recvdata, std::string match_expression, int timeout );

    /* Class member functions */
    private:
      boost::asio::io_service iosrv_; // I/O Service Object
      boost::asio::serial_port serial_socket_; //Serial Communication Interface

    /* Class member objects */
    private:
      std::unique_ptr<boost::asio::serial_port> serial_interface_;

  };

}

#endif