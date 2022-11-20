#include "shipcon/rudder.hh"


namespace shipcon
{
  TwinRudderNode::TwinRudderNode( ros::NodeHandle nh, ros::NodeHandle pnh ):
  nh_( nh ),
  pnh_( pnh )
  {
    pnh_.getParam( "IpAddr", ip_addr_ );
    pnh_.getParam( "Port", port_ );
    pnh_.param<std::string>( "SubnameControlVal", subname_ctrlval_, "controlval_rudder" );
    initEthernet();
    pub_status_ = pnh_.advertise<shipcon::rudder_info>( "rudder_angle", 1 );
    pub_error_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>( "diag_info", 1 );
    sub_ctrlval_ = nh_.subscribe( subname_ctrlval_, 1, &TwinRudderNode::subcallback_ctrl_value, this );

    msg_error_.name = pnh_.getNamespace();
  }

  
  TwinRudderNode::~TwinRudderNode()
  {
    if( threadptr_pub_->joinable() ){ threadptr_pub_->join(); }
    if( threadptr_update_->joinable() ){ threadptr_update_->join(); }
  }


  void TwinRudderNode::run( void )
  {
    threadptr_pub_ = std::make_unique<std::thread>( &TwinRudderNode::thread_publishRudderInfo, this );
    threadptr_update_ = std::make_unique<std::thread>( &TwinRudderNode::thread_updateValue, this );
  }


  void TwinRudderNode::initEthernet( void )
  {
    sock_ = socket( AF_INET, SOCK_DGRAM, 0 );
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons( port_ );
    addr_.sin_addr.s_addr = INADDR_ANY;
    bind( sock_, (struct sockaddr *)&addr_, sizeof(addr_) );
  }


  int TwinRudderNode::receiveUdp( std::string ip, char* data, const int data_length )
  {
    /*Declare and Initialize Local Variables*/
    memset( data, '\0', data_length );
    struct sockaddr_in addr_src;
    int addr_src_len = sizeof( sockaddr_in );
    std::string recv_ip_addr;
    int recv_size = 0;
    
    /*Receive data via UDP*/
    recv_size = recvfrom( sock_, data, data_length, 0, (struct sockaddr *)&addr_src, (socklen_t *)&addr_src_len );
    if( recv_size==-1 ){ return -1; }

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


  int TwinRudderNode::sendUdp( int port, std::string ip, char* data, const int data_length )
  {
    struct sockaddr_in addr_dest;

    addr_dest.sin_family = AF_INET;
    addr_dest.sin_port = htons( port );
    addr_dest.sin_addr.s_addr = inet_addr( ip.c_str() );
    
    return sendto( sock_, data, data_length, 0, (struct sockaddr *)&addr_dest, sizeof(sockaddr_in) );
  }


  void TwinRudderNode::subcallback_ctrl_value( shipcon::rudder_control::ConstPtr value )
  {
    int16_t rudder_angle_port = static_cast<int16_t>( unitcon::angle::rad2deg( value->target_angle_rad[0] ) );
    int16_t rudder_angle_stbd = static_cast<int16_t>( unitcon::angle::rad2deg( value->target_angle_rad[1] ) );
    if( rudder_angle_port > 20 ){ rudder_angle_port = 20; }
    if( rudder_angle_port < -20 ){ rudder_angle_port = -20; }
    if( rudder_angle_stbd > 20 ){ rudder_angle_stbd = 20; }
    if( rudder_angle_stbd < -20 ){ rudder_angle_stbd = -20; }

    char buffer[6];
    memset( buffer, 0, sizeof(buffer) );
    memcpy( &buffer[2], &rudder_angle_port, 2 );
    memcpy( &buffer[4], &rudder_angle_port, 2 );

    int send_size = sendUdp( SELF_PORT, ip_addr_.c_str(), buffer, sizeof(buffer) );
    std::lock_guard<std::mutex> lock( mtx_ );
    ROS_INFO("Sent:%d Byte", send_size);
  }


  void TwinRudderNode::thread_publishRudderInfo( void )
  {
	  char buffer[12];
	
    while( ros::ok() )
    {
      angle_msg_.angle_rad.clear();

      if( receiveUdp( ip_addr_, buffer, sizeof(buffer) ) > 0 )
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        msg_error_.level = msg_error_.OK;
        msg_error_.message = "";

        float port_angle_deg = 0.0;
        float stbd_angle_deg = 0.0;
        memcpy( &port_angle_deg, &buffer[4], 4 );
        memcpy( &stbd_angle_deg, &buffer[8], 4 );
        angle_msg_.angle_rad.push_back( unitcon::angle::deg2rad( static_cast<double>( port_angle_deg ) ) );
        angle_msg_.angle_rad.push_back( unitcon::angle::deg2rad( static_cast<double>( stbd_angle_deg ) ) );
      }
      else
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        msg_error_.level = msg_error_.ERROR;
        msg_error_.message = "Missing_Rudder_Info";
      }
    }
  }


  void TwinRudderNode::thread_updateValue( void )
  {
	  ros::Rate loop_rate( 10 );

    while( ros::ok() )
    {
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        pub_status_.publish( angle_msg_ );
        pub_error_.publish( msg_error_ );
        //ROS_INFO("Rudder Angle PORT:%.1f Deg,  STBD:%.1f Deg", angle_msg_.angle_rad[0]);
      }

      loop_rate.sleep();
    }
  }




}