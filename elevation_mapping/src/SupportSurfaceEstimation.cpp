/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/SupportSurfaceEstimation.hpp"
#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
//#include "elevation_mapping/ElevationMapFunctors.hpp"
//#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
//#include "elevation_mapping/HighGrassElevationMapping.hpp"

//// Grid Map
//#include <grid_map_msgs/GridMap.h>
//#include <grid_map_core/grid_map_core.hpp> // New for high grass
//#include <grid_map_filters/BufferNormalizerFilter.hpp>
//#include <grid_map_cv/grid_map_cv.hpp> // New for high grass

//#include <grid_map_cv/InpaintFilter.hpp> // New for high grass
//#include <opencv/cv.h>
//#include <grid_map_filters/ThresholdFilter.hpp>

//#include <grid_map_cv/InpaintFilter.hpp>
//#include <pluginlib/class_list_macros.h>
//#include <ros/ros.h>

//#include <grid_map_cv/GridMapCvConverter.hpp>
//#include <grid_map_cv/GridMapCvProcessing.hpp>

//// TEST
//#include <any_msgs/State.h>

//// GP Regression
//#include <gaussian_process_regression/gaussian_process_regression.h>

//// Math
//#include <math.h>

//// ROS Logging
//#include <ros/ros.h>

//// Eigen
//#include <Eigen/Dense>

//// TEST thread
//#include <thread>

//// PCL conversions
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/transforms.h>

//// PCL normal estimation
//#include <pcl/features/normal_3d.h>

// ROS msgs
//#include <elevation_mapping/PerformanceAssessment.h>

// File IO
//#include <iostream>
//#include <fstream>

using namespace std;
using namespace grid_map;

namespace elevation_mapping {

SupportSurfaceEstimation::SupportSurfaceEstimation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      //rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy",
      //        "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan",
      //        "sensor_z_at_lowest_scan", "foot_tip_elevation", "support_surface", "elevation_inpainted", "elevation_smooth", "vegetation_height", "vegetation_height_smooth",
      //        "support_surface", "support_surface_smooth", "support_surface_added"}),//, "support_surface_smooth_inpainted", "support_surface_added"}),
      //fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      //supportMap_({"elevation", "variance", "elevation_gp", "elevation_gp_added"}), // New
      supportMapGP_({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}), // New
      //hasUnderlyingMap_(false),
      //visibilityCleanupDuration_(0.0),
      filterChain_("grid_map::GridMap"), // New
      filterChain2_("grid_map::GridMap")
{
    // Timon added foot_tip_elevation layer
  //rawMap_.setBasicLayers({"elevation", "variance"});
  //fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  //supportMap_.setBasicLayers({"elevation", "variance", "elevation_gp", "elevation_gp_added"}); // New
  supportMapGP_.setBasicLayers({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}); // New

  //! uncomment!!
  //clear();

  elevationMapSupportPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_added", 1); // SS
  elevationMapInpaintedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface", 1); // SS
  elevationMapGPPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface_gp", 1); // SS

  nodeHandle_.param("add_old_support_surface_data_to_gp_training", addOldSupportSurfaceDataToGPTraining_, false); // SS
  nodeHandle_.param("weight_terrain_continuity", weightTerrainContinuity_, 1.0); // SS
  nodeHandle_.param("run_terrain_continuity_biasing", runTerrainContinuityBiasing_, true); // SS
  nodeHandle_.param("exponent_sinkage_depth_weight", exponentSinkageDepthWeight_, 2.0); // SS
  nodeHandle_.param("exponent_terrain_continuity_weight", exponentTerrainContinuityWeight_, 2.0); // SS
  nodeHandle_.param("weight_decay_threshold", weightDecayThreshold_, 0.4); // SS
  nodeHandle_.param("exponent_characteristic_value", exponentCharacteristicValue_, 1.0); // SS
  nodeHandle_.param("continuity_filter_gain", continuityFilterGain_, 0.3); // SS

  // For filter chain. // SS
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("/grid_map_filter_chain_one"));
  if(!filterChain_.configure("/grid_map_filter_chain_one", nodeHandle_)){
      std::cout << "Could not configure the filter chain!!" << std::endl;
      return;
  }
  if(!filterChain2_.configure("/grid_map_filter_chain_two", nodeHandle_)){
      std::cout << "INBETWEEN Prob" << std::endl;
      std::cout << "Could not configure the filter chain!!" << std::endl;
      return;
  }

  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);
  supportSurfaceAddingAreaPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("adding_area", 1000);


  supportSurfaceInitializationTrigger_ = false; // SS
  cumulativeSupportSurfaceUncertaintyEstimation_ = 0.0; // SS

  initialTime_ = ros::Time::now();
}

SupportSurfaceEstimation::~SupportSurfaceEstimation()
{
}

bool SupportSurfaceEstimation::updateSupportSurfaceEstimation(std::string tip, GridMap& rawMap){

    // Uncertainty Estimation based on low pass filtered error between foot tip height and predicted support Surface.
    setSupportSurfaceUncertaintyEstimation(tip);

    // Set coefficients spun by three of the foot tips.
    //terrainContinuityBiasing(tip);

    // TODO: these params into config file.
    double tileResolution = 0.08;
    double tileDiameter = 0.23; // HAcked larger, if slow, change this here
    double sideLengthAddingPatch = 1.3;
    //mainGPRegression(tileResolution, tileDiameter, sideLengthAddingPatch, tip);
    return true;
}

//Position3 ElevationMap::getFrontLeftFootTipPosition(){
//    Position3 tipPos(frontLeftFootTip_(0), frontLeftFootTip_(1), frontLeftFootTip_(2));
//    return tipPos;
//}

//Position3 ElevationMap::getFrontRightFootTipPosition(){
//    Position3 tipPos(frontRightFootTip_(0), frontRightFootTip_(1), frontRightFootTip_(2));
//    return tipPos;
//}

//Position3 ElevationMap::getHindLeftFootTipPosition(){
//    Position3 tipPos(hindLeftFootTip_(0), hindLeftFootTip_(1), hindLeftFootTip_(2));
//    return tipPos;
//}

//Position3 ElevationMap::getHindRightFootTipPosition(){
//    Position3 tipPos(hindRightFootTip_(0), hindRightFootTip_(1), hindRightFootTip_(2));
//    return tipPos;
//}

bool SupportSurfaceEstimation::setSupportSurfaceUncertaintyEstimation(std::string tip){

//    Eigen::Vector3f latestTip;
//    if (tip == "left") latestTip = getLatestLeftStance();
//    if (tip == "right") latestTip = getLatestRightStance();

//    // So far not low pass filtered!! -> create Vector for that (TODO)
//    grid_map::Position latestTipPosition(latestTip(0), latestTip(1));
//    double diff = fabs(latestTip(2) - supportMapGP_.atPosition("elevation_gp_added", latestTipPosition));
//    supportSurfaceUncertaintyEstimation_ = diff;
//    if (!isnan(diff)) cumulativeSupportSurfaceUncertaintyEstimation_ += diff;
    return true;
}

double SupportSurfaceEstimation::getFootTipElevationMapDifferenceGP(std::string tip){

//    double radius = 0.1; // Maximum search radius for spiralling search in order to find the closest map element in case if nan is present..
//    Position3 footTip3;
//    if (tip == "left") footTip3 = getFrontLeftFootTipPosition();
//    if (tip == "right") footTip3 = getFrontRightFootTipPosition();
//    grid_map::Position footTipHorizontal(footTip3(0), footTip3(1));
//    double verticalDifference;
//    if (supportMapGP_.exists("smoothed_top_layer_gp")) { // New Check!!!
//        if(supportMapGP_.isInside(footTipHorizontal)){
//            if (!isnan(supportMapGP_.atPosition("smoothed_top_layer_gp", footTipHorizontal))){
//                verticalDifference = footTip3(2) - supportMapGP_.atPosition("smoothed_top_layer_gp", footTipHorizontal); // Hacked to rawMap_
//            }
//            else verticalDifference = getClosestMapValueUsingSpiralIteratorElevation(supportMapGP_, footTipHorizontal, radius, footTip3(2)); // New experiment.. Wrong, not difference yet!!!
//        }
//    }
//    else verticalDifference = std::numeric_limits<double>::quiet_NaN();
//    std::cout << "Vertical Difference!! -> ->: " << verticalDifference << std::endl;
    //return verticalDifference;
}

double SupportSurfaceEstimation::getClosestMapValueUsingSpiralIteratorElevation(grid_map::GridMap& MapReference, Position footTip, double radius, double tipHeight){

//    int counter = 0;
//    for (grid_map::SpiralIterator iterator(MapReference, footTip, radius);
//         !iterator.isPastEnd(); ++iterator) {   // Hacked to is inside..
//        Index index(*iterator);
//        Position pos;
//        MapReference.getPosition(index, pos);
//        if (MapReference.isInside(pos) && !isnan(MapReference.at("smoothed_top_layer_gp", index)) && MapReference.isValid(index)){
//            std::cout << "RETURNED DIFFERENCE TO A CLOSE NEIGHBOR USING SPIRALLING!!!" << std::endl;
//            return tipHeight - MapReference.at("smoothed_top_layer_gp", index); // && MapReference.isValid(index)
//        }
//        counter++;
//        if (counter > 28) break;
//    }
//    return std::numeric_limits<double>::quiet_NaN();
}

} /* namespace */
