#include "shipcon/servo.hh"


namespace shipcon
{
  ServoNode::ServoNode( ros::NodeHandle nh, ros::NodeHandle pnh ):
  nh_( nh ),
  pnh_( pnh )
  {
    pnh_.getParam( "IpAddr", ip_addr_ );
    pnh_.getParam( "Port", port_ );
    pnh_.param<std::string>( "SubnameControlval", subname_ctrlval_, "controlval_servo" );
    initEthernet();

    pub_angle_ = pnh_.advertise<std_msgs::Float32>( "angle", 1 );
    pub_error_ = pnh_.advertise<diagnostic_msgs::DiagnosticStatus>( "diag_info", 1 );
    sub_ctrlval_ = nh_.subscribe( subname_ctrlval_, 1, &ServoNode::subcallback_ctrl_value, this );
    msg_error_.name = "Servo_info_error";
    ctrl_value_msg_.data = 0;
  }


  ServoNode::~ServoNode()
  {
    if( threadptr_pub_->joinable() ){ threadptr_pub_->join(); }
    if( threadptr_update_->joinable() ){ threadptr_update_->join(); }
  }


  void ServoNode::run( void )
  {
    threadptr_pub_ = std::make_unique<std::thread>( &ServoNode::thread_publishServoInfo, this );
    threadptr_update_ = std::make_unique<std::thread>( &ServoNode::thread_updateValue, this );
  }


  void ServoNode::initEthernet( void )
  {
    sock_ = socket( AF_INET, SOCK_DGRAM, 0 );
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons( 50001 );
    addr_.sin_addr.s_addr = INADDR_ANY;
    bind( sock_, (struct sockaddr *)&addr_, sizeof(addr_) );
  }


  int ServoNode::receiveUdp( std::string ip, char* data, const int data_length )
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


  int ServoNode::sendUdp( int port, std::string ip, char* data, const int data_length )
  {
    struct sockaddr_in addr_dest;
    int test;

    addr_dest.sin_family = AF_INET;
    addr_dest.sin_port = htons( port );
    addr_dest.sin_addr.s_addr = inet_addr( ip.c_str() );
    
    return sendto(sock_, data, data_length, 0, (struct sockaddr *)&addr_dest, sizeof(sockaddr_in));
  }


  void ServoNode::subcallback_ctrl_value( const std_msgs::Float32 value )
  {
    char buffer[4];
    memset( buffer, '\0', sizeof(buffer) );
    memcpy( buffer, &value, sizeof(buffer) );

    int send_size = sendUdp( port_, ip_addr_, buffer, sizeof(buffer) );

    std::lock_guard<std::mutex> lock( mtx_ );
    ROS_INFO("Sent:%d Byte", send_size);
  }


  void ServoNode::thread_publishServoInfo( void )
  {
	  ros::Rate loop_rate( 10 );

    while( ros::ok() )
    {
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        pub_angle_.publish( msg_angle_ );
        pub_error_.publish( msg_error_ );
        ROS_INFO( "%.1f[deg]", msg_angle_.data );
      }
      loop_rate.sleep();
    }
  }


  void ServoNode::thread_updateValue( void )
  {
    char buffer[10];
	
    while( ros::ok() )
    {
      if( receiveUdp( ip_addr_, buffer, sizeof(buffer) ) > 0 )
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        msg_error_.level = msg_error_.OK;
        msg_error_.message = "";
        memcpy( &( msg_angle_.data ), &buffer[0], 4 );
      }
      else
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        msg_error_.level = msg_error_.ERROR;
        msg_error_.message = "Servo Data missing";
        msg_angle_.data = 0.0;
      }
    } 
  }


}