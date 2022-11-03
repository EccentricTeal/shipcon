#include "shipcon/angle_converter.hh"

namespace unitcon::angle
{
  double rad2deg( double rad )
  {
    return( ( rad * 180.0 ) / M_PI );
  }

  double deg2rad( double deg )
  {
    return( ( deg * M_PI ) / 180.0 );
  }
}