#include "shipcon/rudder.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_Rudder" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::SingleRudderNode rudder( nh, pnh );
  
  rudder.run();
  ros::spin();
  
  return 0;
}