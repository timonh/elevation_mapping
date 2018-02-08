
 /* ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_mapping/ElevationMapping.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/HighGrassElevationMapping.hpp"

// State Estimator Message
#include "quadruped_msgs/QuadrupedState.h"
#include "quadruped_msgs/Contacts.h"

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Kindr
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>

using namespace std;
using namespace grid_map;
using namespace ros;
using namespace tf;
using namespace pcl;
//using namespace kindr;
//using namespace kindr_ros;

namespace elevation_mapping {

HighGrassElevationMapping::HighGrassElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),
      isContinouslyFusing_(false),
      ignoreRobotMotionUpdates_(false)
{
  ROS_INFO("High Grass elevation mapping node started.");
  readHighGrassParameters();
  // TESTS
  ROS_WARN_STREAM("Elevation mapping topic name: " << pointCloudTopic_);
  //
  highGrassPointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &HighGrassElevationMapping::highGrassPointCloudCallback, this);
  // TODO: Subscribe to foot tip topic! *****************************************************************************************************************************
  // TODO: Check if it is possible to subscribe to less than the state all the time..
  highGrassFootTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &HighGrassElevationMapping::highGrassFootTipStanceCallback, this);

  highGrassElevationMapSubscriber_ = nodeHandle_.subscribe("/elevation_mapping/elevation_map", 1, &HighGrassElevationMapping::highGrassElevationMapCallback, this);

  footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("foot_contact_markers_rviz", 1000);
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("elevation_map_bound_markers_rviz", 1000);

  // Color and shape definition of markers for foot tip ground contact visualization.
  footContactMarkerList_.header.frame_id = "odom";
  footContactMarkerList_.header.stamp = ros::Time();
  footContactMarkerList_.ns = "elevation_mapping";
  footContactMarkerList_.id = 0;
  footContactMarkerList_.type = visualization_msgs::Marker::SPHERE_LIST;
  footContactMarkerList_.action = visualization_msgs::Marker::ADD;
  footContactMarkerList_.pose.orientation.x = 0.0;
  footContactMarkerList_.pose.orientation.y = 0.0;
  footContactMarkerList_.pose.orientation.z = 0.0;
  footContactMarkerList_.pose.orientation.w = 1.0;
  footContactMarkerList_.scale.x = 0.1;
  footContactMarkerList_.scale.y = 0.1;
  footContactMarkerList_.scale.z = 0.1;
  footContactMarkerList_.color.a = 1.0; // Don't forget to set the alpha!
  footContactMarkerList_.color.r = 0.7;
  footContactMarkerList_.color.g = 0.0;
  footContactMarkerList_.color.b = 0.7;


  elevationMapBoundMarkerList_.header.frame_id = "odom";
  elevationMapBoundMarkerList_.header.stamp = ros::Time();
  elevationMapBoundMarkerList_.ns = "elevation_mapping";
  elevationMapBoundMarkerList_.id = 0;
  elevationMapBoundMarkerList_.type = visualization_msgs::Marker::SPHERE_LIST;
  elevationMapBoundMarkerList_.action = visualization_msgs::Marker::ADD;
  elevationMapBoundMarkerList_.pose.orientation.x = 0.0;
  elevationMapBoundMarkerList_.pose.orientation.y = 0.0;
  elevationMapBoundMarkerList_.pose.orientation.z = 0.0;
  elevationMapBoundMarkerList_.pose.orientation.w = 1.0;
  elevationMapBoundMarkerList_.scale.x = 0.1;
  elevationMapBoundMarkerList_.scale.y = 0.1;
  elevationMapBoundMarkerList_.scale.z = 0.1;
  elevationMapBoundMarkerList_.color.a = 1.0; // Don't forget to set the alpha!
  elevationMapBoundMarkerList_.color.r = 0.7;
  elevationMapBoundMarkerList_.color.g = 0.7;
  elevationMapBoundMarkerList_.color.b = 0.7;
}

HighGrassElevationMapping::~HighGrassElevationMapping()
{
}


bool HighGrassElevationMapping::readHighGrassParameters()
{
  // ElevationMapping parameters.

  nodeHandle_.param("point_cloud_topic", pointCloudTopic_, std::string("/points"));

  return true;
}

void HighGrassElevationMapping::highGrassElevationMapCallback(const grid_map_msgs::GridMap& elevationMap)
{

  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(elevationMap, map);


  Position submapCenter1(0.35, -0.18);
  Position submapCenter2(0.35, 0.18);
  Length length(0.1, 0.1);
  bool isSuccess;
  SubmapGeometry geometry1(map, submapCenter1 , length, isSuccess);
  SubmapGeometry geometry2(map, submapCenter2 , length, isSuccess);
  for(grid_map::SubmapIterator it(geometry1); !it.isPastEnd(); ++it){
      //grid_map::Position posnew;
      //map.getPosition(*it,posnew);
      //if(map.at("elevation", *it) == map.at("elevation", *it)){
          //std::cout << "heights: " << map.at("elevation", *it) << std::endl;
          grid_map::Position posnewnew;
          map.getPosition(*it,posnewnew);
          //std::cout << "x: " << posnewnew(0) << std::endl;
          //std::cout << "y: " << posnewnew(1) << std::endl;
          geometry_msgs::Point p;
          p.x = posnewnew(0);
          p.y = posnewnew(1);
          p.z = 0.0;
          elevationMapBoundMarkerList_.points.push_back(p);
          //grid_map::Index indexnewnew;
          //map.getIndex(*it,indexnewnew);
          //std::cout << "index 0: " << indexnewnew(0) << std::endl;
          //std::cout << "index 1: " << indexnewnew(1) << std::endl;

      //}

  }
  for(grid_map::SubmapIterator it(geometry2); !it.isPastEnd(); ++it){
      //grid_map::Position posnew;
      //map.getPosition(*it,posnew);
      //if(map.at("elevation", *it) == map.at("elevation", *it)){
          //std::cout << "heights: " << map.at("elevation", *it) << std::endl;
          grid_map::Position posnewnew;
          map.getPosition(*it,posnewnew);
          //std::cout << "x: " << posnewnew(0) << std::endl;
          //std::cout << "y: " << posnewnew(1) << std::endl;
          geometry_msgs::Point p;
          p.x = posnewnew(0);
          p.y = posnewnew(1);
          p.z = 0.0;
          elevationMapBoundMarkerList_.points.push_back(p);
          //grid_map::Index indexnewnew;
          //map.getIndex(*it,indexnewnew);
          //std::cout << "index 0: " << indexnewnew(0) << std::endl;
          //std::cout << "index 1: " << indexnewnew(1) << std::endl;

      //}

   }
   elevationMapBoundPublisher_.publish(elevationMapBoundMarkerList_);

}

void HighGrassElevationMapping::highGrassFootTipStanceCallback(const quadruped_msgs::QuadrupedState& quadrupedState)
{


//  std::cout << "Callback for Foot tips!!!!!" << std::endl;
//  float state_0 = quadrupedState.contacts[0].state;
  //string name_0 = quadrupedState.contacts[0].name;
//  std::cout << "Quadstate test: " << state_0 << std::endl;
  //std::cout << "Quadname test: " << name_0 << std::endl;


//  float state_1 = quadrupedState.contacts[1].state;
  //string name_1 = quadrupedState.contacts[1].name;

//  std::cout << "Quadstate test: " << state_1 << std::endl;
  //std::cout << "Quadname test: " << name_1 << std::endl;


//  float state_2 = quadrupedState.contacts[2].state;
//  string name_2 = quadrupedState.contacts[2].name;

//  std::cout << "Quadstate test: " << state_2 << std::endl;
//  std::cout << "Quadstate test: " << name_2 << std::endl;


//  float state_3 = quadrupedState.contacts[3].state;
//  string name_3 = quadrupedState.contacts[3].name;

//  std::cout << "Quadstate test: " << state_3 << std::endl;
//  std::cout << "Quadstate test: " << name_3 << std::endl;


  // TODO: Assign feet center:
  //feetcenter_(0) = quadrupedState.frame_transforms.at("feetcenter").transform.translation.x;

  // Set class variables.
  LFTipPostiion_(0) = quadrupedState.contacts[0].position.x;
  LFTipPostiion_(1) = quadrupedState.contacts[0].position.y;
  LFTipPostiion_(2) = quadrupedState.contacts[0].position.z;
  RFTipPostiion_(0) = quadrupedState.contacts[1].position.x;
  RFTipPostiion_(1) = quadrupedState.contacts[1].position.y;
  RFTipPostiion_(2) = quadrupedState.contacts[1].position.z;

  for(unsigned int i; i <= 3; ++i){
      geometry_msgs::Point p;
      p.x = quadrupedState.contacts[i].position.x;
      p.y = quadrupedState.contacts[i].position.y;
      p.z = quadrupedState.contacts[i].position.z;




      if(quadrupedState.contacts[i].state == 1){
          footContactMarkerList_.points.push_back(p);

      }

      // Clear some foot tip markers after a certain time (and move them to the beginning of the list, as no pop_front exists)
      int size =footContactMarkerList_.points.size();
      if(size > 1200){
          for(unsigned int j = 0; j<400; ++j){
              footContactMarkerList_.points[j] = footContactMarkerList_.points[size - 400 + j];
          }
          for (unsigned int n = 0; n < (size-400); ++n){
              footContactMarkerList_.points.pop_back();
          }
      }

  }
  footContactPublisher_.publish(footContactMarkerList_);


}


void HighGrassElevationMapping::highGrassPointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{

}


} /* namespace */

