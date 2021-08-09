#ifndef SHIPCON__HARDWARE_INTERFACE__UDPCOM_HH
#define SHIPCON__HARDWARE_INTERFACE__UDPCOM_HH

/**
 * @file udpcom.hh
 * @brief Simple class library for UDP communication(Send or Receive) in C++ (Not Thread Safe)
 * @author Suisei WADA
 * @date 20th November, 2020
 * 
 * @details
 * @note
 */

#include <string>
#include <memory>
#include <vector>
#include <array>
#include <tuple>
#include <boost/asio.hpp>


/**
 * Core class for UDP communication.
 */
namespace shipcon::hwcom
{
  class UdpTransmitter
  {       
    /* Constructor, Destructor */
    public:
      UdpTransmitter( std::string dest_ip, uint16_t dest_port );
      ~UdpTransmitter();

    /* Public Member Functions */
    public:
      void setDestEndpoint(  std::string dest_ip, uint16_t dest_port );
      size_t sendData( std::string send_data );    

    /* Class member functions */
    private:

    /* Class member objects */
    private:
      boost::asio::ip::udp::endpoint dest_endpoint_;
      std::unique_ptr<boost::asio::io_service> iosrv_ptr_;
      std::unique_ptr<boost::asio::ip::udp::socket> socket_ptr_;
  };


  class UdpReceiver
  { 
    /* Constructor, Destructor */
    public:
      UdpReceiver( uint16_t udp_port );
      ~UdpReceiver();

    /* Public member functions */
    public:
      std::tuple<size_t, std::string> recvData( std::string src_ip );

    /* Class member functions */
    private:
      void init_( void );
      std::vector<char> recvdata_;

    /* Class member variables */
    private:
      std::unique_ptr<boost::asio::io_service> iosrv_ptr_;
      std::unique_ptr<boost::asio::ip::udp::socket> socket_ptr_;
  };
}

#endif



