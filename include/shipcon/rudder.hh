#ifndef SHIPCON__RUDDER__HH
#define SHIPCON__RUDDER__HH

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>
#include <string>
#include <mutex>
#include <memory>

namespace shipcon
{
  class SingleRudderNode
  {
    /** Constants **/
    private:
      
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
      ros::Publisher pub_status_; 
      ros::Publisher pub_error_;
      ros::Subscriber sub_ctrlval_;
      std::string subname_ctrlval_;

      diagnostic_msgs::DiagnosticStatus msg_error_;
      std_msgs::Int16 ctrl_value_msg_;
      std_msgs::Float32 angle_msg_;

      //Thread
      std::unique_ptr<std::thread> threadptr_pub_;
      std::unique_ptr<std::thread> threadptr_update_;

    /** Constrctor, Destructor **/
    public:
      SingleRudderNode( ros::NodeHandle nh, ros::NodeHandle pnh );
      ~SingleRudderNode();

    /** Methods **/
    public:
      void run( void );

    private:
      void initEthernet( void );
      int receiveUdp( std::string ip, char* data, const int data_length );
      int sendUdp( int port, std::string ip, char* data, const int data_length );

    /** Callback **/
    private:
      void subcallback_ctrl_value( std_msgs::Int16::ConstPtr value );

    /** Thread **/
    private:
      void thread_publishRudderInfo( void );
      void thread_updateValue( void );
  };
}



#endif