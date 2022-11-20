#include "shipcon/radio_control.hh"

namespace shipcon
{
  RadioControlNode::RadioControlNode( ros::NodeHandle nh, ros::NodeHandle pnh ):
  nh_( nh ),
  pnh_( pnh )
  {
    pnh_.getParam( "IpAddr", ip_addr_ );
    pnh_.getParam( "Port", port_ );
    pnh_.param<std::string>( "SubnameControlval", subname_ctrlval_, "controlval_motor" );
    initEthernet();

    pub_rcvalue_motor_ = pnh_.advertise<std_msgs::Int16>( "motor_value", 1 );
    pub_rcvalue_prop_pitch_ = pnh_.advertise<std_msgs::Int16>( "prop_pitch_value", 1 );
    pub_rcvalue_rudder_ = pnh_.advertise<std_msgs::Int16>( "rudder_value", 1 );
    pub_rcvalue_mode_ = pnh_.advertise<std_msgs::Int16>( "mode_value", 1 );
    pub_rcvalue_gyro_reset_ = pnh_.advertise<std_msgs::Int16>( "gyro_reset_value", 1 );
    pub_rcvalue_diaginfo_ = pnh_.advertise<diagnostic_msgs::DiagnosticStatus>( "diag_info", 1 );

    msg_error_.name = "RadioControl_info_error";
  }


  RadioControlNode::~RadioControlNode()
  {
    if( threadptr_pub_->joinable() ){ threadptr_pub_->join(); }
    if( threadptr_update_->joinable() ){ threadptr_update_->join(); }
  }


  void RadioControlNode::run()
  {
    threadptr_pub_ = std::make_unique<std::thread>( &RadioControlNode::thread_publish, this );
    threadptr_update_ = std::make_unique<std::thread>( &RadioControlNode::thread_update, this );
  }


  void RadioControlNode::initEthernet( void )
  {
    sock_ = socket( AF_INET, SOCK_DGRAM, 0 );
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons( 50000 );
    addr_.sin_addr.s_addr = INADDR_ANY;
    bind( sock_, (struct sockaddr *)&addr_, sizeof( addr_ ) );  
  }


  void RadioControlNode::thread_publish( void )
  {
    ros::Rate loop_rate(10);

    while( ros::ok() )
    { 
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        pub_rcvalue_motor_.publish( msg_motor_ );
        pub_rcvalue_prop_pitch_.publish( msg_prop_pitch_ );
        pub_rcvalue_rudder_.publish( msg_rudder_ );
        pub_rcvalue_mode_.publish( msg_mode_ );
        pub_rcvalue_gyro_reset_.publish( msg_gyro_reset_ );
        pub_rcvalue_diaginfo_.publish( msg_error_ );
        ROS_INFO( "Throttle:%d, Pitch:%d, Rudder:%d, AutoFlag:%d, GyroSW:%d", msg_motor_.data, msg_prop_pitch_.data, msg_rudder_.data, msg_mode_.data, msg_gyro_reset_.data );
      }

	    loop_rate.sleep();
	  }
  }


  void RadioControlNode::thread_update( void )
  {
    char in_buf[16];
    struct sockaddr_in addr_src;
    const int addr_src_len = sizeof( sockaddr_in );

    while( ros::ok() )
    {
      memset( in_buf, 0, sizeof(in_buf) );
      memset( &addr_src, '\0', sizeof(addr_src) );

      if( recvfrom( sock_, in_buf, sizeof(in_buf), 0, (struct sockaddr *)&addr_src, (socklen_t *)&addr_src_len ) == -1 )
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        msg_motor_.data = 0;
        msg_prop_pitch_.data = 0;
        msg_rudder_.data = 0;
        msg_mode_.data = 0;
        msg_gyro_reset_.data = 0;
	      msg_error_.level = msg_error_.ERROR;
      }

      if( inet_ntoa( addr_src.sin_addr ) == ip_addr_ )
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        memcpy( &(msg_motor_.data), &in_buf[0], 2 );
        memcpy( &(msg_prop_pitch_.data), &in_buf[14], 2 );
        memcpy( &(msg_rudder_.data),&in_buf[2], 2 );
        msg_mode_.data = in_buf[8];
        msg_gyro_reset_.data = in_buf[10];
        msg_error_.level = msg_error_.OK;
      }
    }
  }



}