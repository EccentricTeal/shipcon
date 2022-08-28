#ifndef SHIPCON__BOWTHRUSTER__HH
#define SHIPCON__BOWTHRUSTER__HH

#include "ros/ros.h"
#include "shipcon/bowthruster_control.h"
#include "shipcon/bowthruster_info.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>
#include <string>
#include <mutex>
#include <memory>

namespace shipcon
{
  class BowThrusterNode
  {
    /** Constants **/
    private:
      const double MAX_BOWTHRUSTER_RPM = 3000.0;
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
      ros::Publisher pub_info_;
      ros::Subscriber sub_ctrlval_;
      std::string subname_ctrlval_;
      shipcon::bowthruster_info msg_info_;
      shipcon::bowthruster_control msg_ctrl_value_;

      //Thread
      std::unique_ptr<std::thread> threadptr_pub_;
      std::unique_ptr<std::thread> threadptr_update_;

    /** Constrctor, Destructor **/
    public:
      BowThrusterNode( ros::NodeHandle nh, ros::NodeHandle pnh );
      ~BowThrusterNode();

    /** Methods **/
    public:
      void run( void );

    private:
      void initEthernet( void );
      int receiveUdp( std::string ip, char* data, const int data_length );
      int sendUdp( int port, std::string ip, char* data, const int data_length );

    /** Callback **/
    private:
      void subcallback_ctrl_value( shipcon::bowthruster_control::ConstPtr msg );

    /** Thread **/
    private:
      void thread_publishBowThrusterInfo( void );
      void thread_updateBowThrusterInfo( void );
  };
}



#endif