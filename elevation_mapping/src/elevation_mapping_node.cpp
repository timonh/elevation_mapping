/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/HighGrassElevationMapping.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_mapping");
  ros::NodeHandle nodeHandle("~");
  elevation_mapping::ElevationMapping elevationMap(nodeHandle);
  elevation_mapping::HighGrassElevationMapping highGrassElevationMap(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(2); // Use n threads // MANIPULATED!!!!!
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
