#ifndef SHIPCON__GYRO_JAE_JG35FD__HH
#define SHIPCON__GYRO_JAE_JG35FD__HH

//ROS Packages
#include <ros/ros.h>
#include "shipcon/serialcom.hh"
#include "shipcon/angle_converter.hh"

//ROS Service
#include "shipcon/Jg35fdCalibrateBiasDrift.h"
#include "shipcon/Jg35fdControlCalculate.h"
#include "shipcon/Jg35fdControlOutput.h"
#include "shipcon/Jg35fdResetAngle.h"
#include "shipcon/Jg35fdSetAnalogMode.h"

//ROS Publisher
#include "shipcon/gyro.h"
//STL
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <vector>
#include <cmath>
#include <utility>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/regex.hpp>

namespace shipcon
{
  class GyroJg35fd
  {
    /* Constants */
    private:
      const int BAUDRATE = 9600;
      boost::regex REGEX_CONDITION_HEADER = boost::regex("\x02[\x81-\x84]");
      const std::string DEVELOPPER_NAME = "JAPAN AERONAUTICAL ELECTRONICS CO., LTD.";
      const std::string DEVICE_TYPE = "YAW GYRO JG35FD";

    /* Private Member Objects*/
    private:
      //Serial Communication
      std::unique_ptr<hwcomlib::SerialCom> serialif_;
      std::string serial_port_name_;
      
      //ROS Service
      ros::ServiceServer srv_control_output_;
      ros::ServiceServer srv_calibrate_bias_drift_;
      ros::ServiceServer srv_control_calculate_;
      ros::ServiceServer srv_reset_angle_;
      ros::ServiceServer srv_set_analog_mode_;

      //ROS Publisher
      ros::Publisher pub_gyro_info_;

      //ROS Node
      ros::NodeHandle nh_, pnh_;
      
      //Buffers
      boost::asio::streambuf recv_buffer_;
      std::vector<unsigned char> data_buffer_;
      //Utility
      std::mutex mtx_;

    /* Constructor, Destructor */
    public:
      GyroJg35fd( ros::NodeHandle nh, ros::NodeHandle pnh );
      ~GyroJg35fd();

    /* Private Methods */
    private:
      //ROS Service
      void initService( void );

      //ROS Publisher
      void initPublisher( void );

      //Serial Communication
      void initSerialParameter( void );
      bool initSerial( void );
      bool startSerial( void );
      void callback_sendSerial( const boost::system::error_code& ec, std::size_t sendsize );
      void callback_receive_header( const boost::system::error_code& ec, std::size_t recvsize );
      void callback_receive_data( const boost::system::error_code& ec, std::size_t recvsize, unsigned int datasize );
      void updateData( void );

      //Gyro Applications
      void configureOutput( uint8_t interval, uint8_t mode );
      void requestConfigureBiasDrift( uint8_t request );
      void configureCalculate( uint8_t request );
      void requestResetAngle( double new_angle );
      void configureAnalog( uint8_t mode, uint8_t range );

      //Callback
      bool callback_srv_control_output(
        shipcon::Jg35fdControlOutput::Request &req,
        shipcon::Jg35fdControlOutput::Response &res
      );
      bool callback_srv_calibrate_bias_drift(
        shipcon::Jg35fdCalibrateBiasDrift::Request &req,
        shipcon::Jg35fdCalibrateBiasDrift::Response &res
      );
      bool callback_srv_control_calculate(
        shipcon::Jg35fdControlCalculate::Request &req,
        shipcon::Jg35fdControlCalculate::Response &res
      );
      bool callback_srv_reset_angle(
        shipcon::Jg35fdResetAngle::Request &req,
        shipcon::Jg35fdResetAngle::Response &res
      );
      bool callback_srv_set_analog_mode(
        shipcon::Jg35fdSetAnalogMode::Request &req,
        shipcon::Jg35fdSetAnalogMode::Response &res
      );
  };
}

#endif