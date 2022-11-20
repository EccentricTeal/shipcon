#include "shipcon/rudder.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_TwinRudder" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::TwinRudderNode rudder( nh, pnh );
  
  rudder.run();
  ros::spin();
  
  return 0;
}