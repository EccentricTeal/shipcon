#include "shipcon/bowthruster.hh"


namespace shipcon
{
  BowThrusterNode::BowThrusterNode( ros::NodeHandle nh, ros::NodeHandle pnh ):
  nh_( nh ),
  pnh_( pnh )
  {
    pnh_.getParam( "IpAddr", ip_addr_ );
    pnh_.getParam( "Port", port_ );
    pnh_.param<std::string>( "SubnameControlval", subname_ctrlval_, "controlval_bowthruster" );
    initEthernet();

    pub_info_ = pnh_.advertise<shipcon::bowthruster_info>( "info", 1 );
    sub_ctrlval_ = nh_.subscribe( subname_ctrlval_, 1, &BowThrusterNode::subcallback_ctrl_value, this );
  }


  BowThrusterNode::~BowThrusterNode()
  {
    if( threadptr_pub_->joinable() ){ threadptr_pub_->join(); }
    if( threadptr_update_->joinable() ){ threadptr_update_->join(); }
  }


  void BowThrusterNode::run( void )
  {
    threadptr_pub_ = std::make_unique<std::thread>( &BowThrusterNode::thread_publishBowThrusterInfo, this );
    threadptr_update_ = std::make_unique<std::thread>( &BowThrusterNode::thread_updateBowThrusterInfo, this );
  }


  void BowThrusterNode::initEthernet( void )
  {
    sock_ = socket( AF_INET, SOCK_DGRAM, 0 );
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons( port_ );
    addr_.sin_addr.s_addr = INADDR_ANY;
    bind( sock_, (struct sockaddr *)&addr_, sizeof(addr_) );
  }


  int BowThrusterNode::receiveUdp( std::string ip, char* data, const int data_length )
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


  int BowThrusterNode::sendUdp( int port, std::string ip, char* data, const int data_length )
  {
    struct sockaddr_in addr_dest;
    int test;

    addr_dest.sin_family = AF_INET;
    addr_dest.sin_port = htons( port );
    addr_dest.sin_addr.s_addr = inet_addr( ip.c_str() );
    
    return sendto(sock_, data, data_length, 0, (struct sockaddr *)&addr_dest, sizeof(sockaddr_in));
  }


  void BowThrusterNode::subcallback_ctrl_value( shipcon::bowthruster_control::ConstPtr msg )
  {
    /* Declaration */
    int16_t throttle = 0;
    char buffer[4];
    memset( buffer, '\0', sizeof(buffer) );

    /* Data conversion */
    throttle = static_cast<int16_t>( msg->target_rpm * 100.0 / MAX_BOWTHRUSTER_RPM );
    if( throttle > 100 ){ throttle = 100; }
    if( throttle < -100 ){ throttle = -100; }

    /* Data set */
    memcpy( &throttle, &buffer[0], 2 );

    /* Send UDP */
    int send_size = sendUdp( SELF_PORT, ip_addr_, buffer, sizeof(buffer) );

    // std::lock_guard<std::mutex> lock( mtx_ );
    // ROS_INFO("Sent:%d Byte", send_size);
  }


  void BowThrusterNode::thread_publishBowThrusterInfo( void )
  {
	  ros::Rate loop_rate( 10 );

    while( ros::ok() )
    {
      {
        std::lock_guard<std::mutex> lock( mtx_ );
        pub_info_.publish( msg_info_ );
        // ROS_INFO( "%.1f[rpm]", msg_info_.rpm );
      }
      loop_rate.sleep();
    }
  }


  void BowThrusterNode::thread_updateBowThrusterInfo( void )
  {
    char buffer[10];
	
    while( ros::ok() )
    {
      int16_t rpm = 0;
      float current = 0.0;
      float temperature = 0.0;

      if( receiveUdp( ip_addr_, buffer, sizeof(buffer) ) > 0 )
      {
        memcpy( &rpm, &buffer[0], 2 );
        memcpy( &current, &buffer[2], 4 );
        memcpy( &temperature, &buffer[6], 4 );

        std::lock_guard<std::mutex> lock( mtx_ );
        msg_info_.rpm = static_cast<double>( rpm );
        msg_info_.current = static_cast<double>( current );
        msg_info_.temperature = static_cast<double>( temperature );
      }
      else
      {
        ;
      }
    } 
  }


}