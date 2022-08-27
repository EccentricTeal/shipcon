#include "shipcon/radio_control.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_RadioControl" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::RadioControlNode rc( nh, pnh );
  
  rc.run();
  ros::spin();
  
  return 0;
}