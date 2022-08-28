#ifndef SHIPCON__MOTOR__HH
#define SHIPCON__MOTOR__HH

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "shipcon/motor_control.h"
#include "shipcon/motor_info.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>
#include <string>
#include <mutex>
#include <memory>

namespace shipcon
{
  class MotorNode
  {
    /** Constants **/
    private:
      const double MAX_MOTOR_RPM = 2000.0;
      const double MAX_PITCH_DEG = 20.0;
      const int SELF_PORT = 50000;
      
    /** Member Objects **/
    private:
      //Thread
      std::mutex mtx_;

      //Socket Com
      int sock_;
      struct sockaddr_in addr_;
      std::string ip_addr_;
      int port_;

      //ROS
      ros::NodeHandle nh_, pnh_;
      ros::Publisher pub_motor_info_;
      ros::Subscriber sub_ctrlval_;
      std::string subname_ctrlval_;
      shipcon::motor_info msg_status_;

      //Thread
      std::unique_ptr<std::thread> threadptr_pub_;
      std::unique_ptr<std::thread> threadptr_update_;

    /** Constrctor, Destructor **/
    public:
      MotorNode( ros::NodeHandle nh, ros::NodeHandle pnh );
      ~MotorNode();

    /** Methods **/
    public:
      void run( void );

    private:
      void initEthernet( void );
      int receiveUdp( std::string ip, char* data, const int data_length );
      int sendUdp( int port, std::string ip, char* data, const int data_length );

    /** Callback **/
    private:
      void subcallback_ctrl_value( shipcon::motor_control::ConstPtr msg );

    /** Thread **/
    private:
      void thread_publishMotorInfo( void );
      void thread_updateValue( void );

    /** Math calc **/
    double deg2rad( double deg ){ return ( deg * M_PI ) / 180.0; }
    double rad2deg( double rad ){ return ( rad * 180.0 ) / M_PI; }
  };
}



#endif