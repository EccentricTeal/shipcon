#ifndef SHIPCON__MOTOR__HH
#define SHIPCON__MOTOR__HH

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
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
      const std::string MOTOR_IP;
      const int MOTOR_PORT;

    /** Member Objects **/
    private:
      //Thread
      std::mutex mtx_;

      //Socket Com
      int sock_;
      struct sockaddr_in addr_;

      //ROS
      ros::NodeHandle nh_;
      ros::Publisher pub_status_; 
      ros::Publisher pub_error_;
      ros::Subscriber sub_ctrl_;
      shipcon::motor_info msg_status_;
      diagnostic_msgs::DiagnosticStatus msg_error_;
      std_msgs::Int16 ctrl_value_msg_;

      //Thread
      std::unique_ptr<std::thread> threadptr_pub_;
      std::unique_ptr<std::thread> threadptr_update_;

    /** Constrctor, Destructor **/
    public:
      MotorNode( std::string ip, int port );
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
      void subcallback_ctrl_value( const std_msgs::Int16 value );

    /** Thread **/
    private:
      void thread_publishMotorInfo( void );
      void thread_updateValue( void );
  };
}



#endif