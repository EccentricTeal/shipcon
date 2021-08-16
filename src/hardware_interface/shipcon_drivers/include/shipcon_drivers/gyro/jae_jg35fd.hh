#ifndef SHIPCON__HARDWARE_INTERFACE__SHIPCON_DRIVERS__GYRO__JAE_JG35FD_HH
#define SHIPCON__HARDWARE_INTERFACE__SHIPCON_DRIVERS__GYRO__JAE_JG35FD_HH

/**
 * @file jae_jg35fd.hh
 * @brief ROS2 Driver for JAE FOG JG35FD
 * @author Suisei WADA
 * @date 14th August, 2021
 * 
 * @details
 * @note
 */

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <iostream>
#include <boost/asio.hpp>

/**
 * Core class for Serial communication.
 */
namespace shipcon::shipcon_drivers::gyro
{
  class DriverNodeJAEJG35FD : public rclcpp::Node
  {
    /*** Constructors and Destructros ***/
    public:
      DriverNodeJAEJG35FD( std::string node_name, std::string name_space, std::string port_path );
      ~DriverNodeJAEJG35FD();

    /*** Public Methods ***/
    public:
    
    /*** Private Methods ***/
    private:
      //Initializing
      void initSerialCom( unsigned int baudrate_kbps );
      //Serial transceiving
      void sendSerial();
      void receiveSerial();


    /*** Private Member Objects ***/
    private:
      //Serial Communication( Boost::Asio )
      boost::asio::io_service iosrv_;
      boost::asio::serial_port serial_socket_;
      //Data



  };
}


#endif