#include "shipcon/bowthruster.hh"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "NODE_BowThruster" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  shipcon::BowThrusterNode bowthruster( nh, pnh );
  
  bowthruster.run();
  ros::spin();
  
  return 0;
}