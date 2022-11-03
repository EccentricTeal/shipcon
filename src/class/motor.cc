#include "shipcon/motor.hh"


namespace shipcon
{
  MotorNode::MotorNode( ros::NodeHandle nh, ros::NodeHandle pnh ):
  nh_( nh ),
  pnh_( pnh )
  {
    pnh_.getParam( "IpAddr", ip_addr_ );
    pnh_.getParam( "Port", port_ );
    pnh_.param<std::string>( "SubnameControlVal", subname_ctrlval_, "controlval_motor" );
    initEthernet();

    pub_motor_info_ = pnh_.advertise<shipcon::motor_info>( "status", 1 );
    sub_ctrlval_ = nh_.subscribe( subname_ctrlval_, 1, &MotorNode::subcallback_ctrl_value, this );
  }


  MotorNode::~MotorNode()
  {
    if( threadptr_pub_->joinable() ){ threadptr_pub_->join(); }
    if( threadptr_update_->joinable() ){ threadptr_update_->join(); }
  }


  void MotorNode::run( void )
  {
    threadptr_pub_ = std::make_unique<std::thread>( &MotorNode::thread_publishMotorInfo, this );
    threadptr_update_ = std::make_unique<std::thread>( &MotorNode::thread_updateValue, this );
  }


  void MotorNode::initEthernet( void )
  {
    sock_ = socket( AF_INET, SOCK_DGRAM, 0 );
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons( port_ );
    addr_.sin_addr.s_addr = INADDR_ANY;
    bind( sock_, (struct sockaddr *)&addr_, sizeof(addr_) );
  }


  int MotorNode::receiveUdp( std::string ip, char* data, const int data_length )
  {
    /*Declare and Initialize Local Variables*/
    memset( data, '\0', data_length );
    struct sockaddr_in addr_src;
    int addr_src_len = sizeof( sockaddr_in );
    std::string recv_ip_addr;
    int recv_size = 0;
    
    /*Receive data via UDP*/
    recv_size = recvfrom( sock_, data, data_length, 0, (struct sockaddr *)&addr_src, (socklen_t *)&addr_src_len );
    if( recv_size == -1 ){ return -1; }

    /*Evaluate Source IP*/
	  recv_ip_addr = inet_ntoa( addr_src.sin_addr );

    if( recv_ip_addr == ip )
    {
      return recv_size;
    }
    else
    {
      memset( data, 0, sizeof(data) );
      return 0;
    }
  }


  int MotorNode::sendUdp( int port, std::string ip, char* data, const int data_length )
  {
    struct sockaddr_in addr_dest;
    int test;

    addr_dest.sin_family = AF_INET;
    addr_dest.sin_port = htons( port );
    addr_dest.sin_addr.s_addr = inet_addr( ip.c_str() );
    
    return sendto(sock_, data, data_length, 0, (struct sockaddr *)&addr_dest, sizeof(sockaddr_in));
  }


  void MotorNode::subcallback_ctrl_value( shipcon::motor_control::ConstPtr msg )
  {
    /** Declaration **/
    int16_t throttle = 0; //-100...100
    int16_t pitch = 0; //-100...100
    char buffer[4];
    memset( buffer, '\0', sizeof(buffer) );

    /** Data conversion **/
    throttle = static_cast<int16_t>( msg->target_rpm * 100.0 / MAX_MOTOR_RPM );
    if( throttle > 100 ){ throttle = 100; }
    if( throttle < -100 ){ throttle = -100; }
    pitch = static_cast<int16_t>( rad2deg( msg->target_pitch_rad ) * 100.0 / MAX_MOTOR_RPM );
    if( pitch > 100 ){ pitch = 100; }
    if( pitch < -100 ){ pitch = -100; }

    /** Buffer set **/
    memcpy( &throttle, &buffer[0], 2 );
    memcpy( &pitch, &buffer[2], 2 );

    /** Send UDP **/
    int send_size = sendUdp( SELF_PORT, ip_addr_, buffer, sizeof(buffer) );

    std::lock_guard<std::mutex> lock( mtx_ );
    ROS_INFO("Sent:%d Byte", send_size);
  }


  void MotorNode::thread_publishMotorInfo( void )
  {
	  ros::Rate loop_rate( 10 );

    while( ros::ok() )
    {
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        pub_motor_info_.publish( msg_status_ );
        ROS_INFO("%.1f[rpm]  %.1f[A]  %.1f[rad]", msg_status_.rpm, msg_status_.current, msg_status_.pitch_rad );
      }

      loop_rate.sleep();
    }
  }


  void MotorNode::thread_updateValue( void )
  {
    char buffer[10];
	
    while( ros::ok() )
    {
      if( receiveUdp( ip_addr_, buffer, sizeof(buffer) ) > 0 )
      {
        int16_t rpm = 0.0;
        float current = 0.0;
        float pitch = 0.0;

        memcpy( &rpm, &buffer[0], 2 );
        memcpy( &current, &buffer[2], 4 );
        memcpy( &pitch, &buffer[6], 4 );

        std::lock_guard<std::mutex> lock( mtx_ );
        msg_status_.rpm = static_cast<double>( rpm );
        msg_status_.current = static_cast<double>( current );
        msg_status_.pitch_rad = deg2rad( static_cast<double>( pitch ) );
      }
      else
      {
        ;
      }
    } 
  }


}