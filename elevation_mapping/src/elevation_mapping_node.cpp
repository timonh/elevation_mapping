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

// New.
#include "elevation_mapping/StanceProcessor.hpp"

#include <ros/console.h>

int main(int argc, char** argv)
{
    {
    ros::init(argc, argv, "elevation_mapping");
    ros::NodeHandle nodeHandle("~");
    elevation_mapping::ElevationMapping elevationMap(nodeHandle);

    // Added by timon to set the logger level.
//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
//       ros::console::notifyLoggerLevelsChanged();
//    }

    // Spin
    ros::AsyncSpinner spinner(2); // Use n threads // MANIPULATED!!!!!
    spinner.start();

    ros::waitForShutdown();
    }
  return 0;
}
