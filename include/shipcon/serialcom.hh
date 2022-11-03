#ifndef HARDWARE_COMMUNICATION_LIB__SERIALCOM_HH
#define HARDWARE_COMMUNICATION_LIB__SERIALCOM_HH

/**
 * @file serialcom.hh
 * @brief Simple class library for Serial communication(Send or Receive) in C++ (Not Thread Safe)
 * @author Suisei WADA
 * @date 25th July, 2021
 * 
 * @details
 * @note
**/

#include <string>
#include <iostream>
#include <memory>
#include <thread>
#include <functional>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <streambuf>

/**
 * Core class for Serial communication.
**/
namespace hwcomlib
{

  class SerialCom
  {
    /* typedef */
    typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> Iterator;

    /* Constructor, Destructor */
    public:
      SerialCom( std::string port, unsigned int baudrate );
      ~SerialCom();

    /* Public Mthods, Accessor */
    public:
      //Configuration
      bool setPort( std::string port );
      bool setBaudRate( unsigned int rate );
      bool setCharacterSize( unsigned int size );
      bool setFlowControl( boost::asio::serial_port_base::flow_control::type option );
      bool setParity( boost::asio::serial_port_base::parity::type option );
      bool setStopBits( boost::asio::serial_port_base::stop_bits::type option );

      //Control
      void run( void );
      void stop( void );

      //Dispatch send/write task
      void dispatchSend( 
        std::vector<unsigned char>& buffer,
        std::function<void( const boost::system::error_code&, std::size_t )> handler
      );
      void dispatchSend( 
        std::shared_ptr<std::vector<unsigned char>> buffer,
        std::function<void( const boost::system::error_code&, std::size_t )> handler
      );
      void dispatchRecv(
        boost::asio::streambuf& buffer,
        std::function<void( const boost::system::error_code&, std::size_t )> handler
      );
      void dispatchRecvSize(
        boost::asio::streambuf& buffer,
        unsigned int size,
        std::function<void( const boost::system::error_code&, std::size_t )> handler
      );
      void dispatchRecvUntil(
        boost::asio::streambuf& buffer,
        std::string delimiter,
        std::function<void( const boost::system::error_code&, std::size_t )> handler
      );
      void dispatchRecvUntil(
        boost::asio::streambuf& buffer,
        const boost::regex& regex_condition,
        std::function<void( const boost::system::error_code&, std::size_t )> handler
      );

    private:
      std::pair<Iterator, bool> match_condition( Iterator begin, Iterator end );


    /* Class member objects */
    private:
      boost::asio::io_service iosrv_;
      std::unique_ptr<boost::asio::io_service::work> iosrv_work_;
      std::unique_ptr<boost::asio::serial_port> serialport_;
      std::unique_ptr<std::thread> thread_async_serialcom_;
  };

}

#endif