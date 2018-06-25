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
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/HighGrassElevationMapping.hpp"

// Grid Map
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp> // New for high grass
#include <grid_map_filters/BufferNormalizerFilter.hpp>
#include <grid_map_cv/grid_map_cv.hpp> // New for high grass

#include <grid_map_cv/InpaintFilter.hpp> // New for high grass
#include <opencv/cv.h>
#include <grid_map_filters/ThresholdFilter.hpp>

#include <grid_map_cv/InpaintFilter.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>

// TEST
#include <any_msgs/State.h>

// GP Regression
#include <gaussian_process_regression/gaussian_process_regression.h>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

// TEST thread
#include <thread>

// PCL conversions
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// PCL normal estimation
#include <pcl/features/normal_3d.h>

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
      supportMap_({"elevation", "variance", "elevation_gp", "elevation_gp_added"}), // New
      supportMapGP_({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}), // New
      //hasUnderlyingMap_(false),
      //visibilityCleanupDuration_(0.0),
      filterChain_("grid_map::GridMap"), // New
      filterChain2_("grid_map::GridMap")
{
    // Timon added foot_tip_elevation layer
  //rawMap_.setBasicLayers({"elevation", "variance"});
  //fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  supportMap_.setBasicLayers({"elevation", "variance", "elevation_gp", "elevation_gp_added"}); // New
  supportMapGP_.setBasicLayers({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}); // New

  //! uncomment!!
  //clear();

  // TEST:
  //elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);
  //elevationMapSupportPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_added", 1);
  //elevationMapInpaintedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface", 1);
  elevationMapGPPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface_gp", 1);
  // END TEST

  //elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  //elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  //if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
  //    nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
  // TODO if (enableVisibilityCleanup_) when parameter cleanup is ready.
  //visbilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

  // Launching parameters.
  nodeHandle_.param("run_drift_adjustment", driftAdjustment_, true);
  nodeHandle_.param("apply_frame_correction", applyFrameCorrection_, true);
  nodeHandle_.param("kp", kp_, 0.24);
  nodeHandle_.param("ki", ki_, 0.67);
  nodeHandle_.param("kd", kd_, -0.07);
  nodeHandle_.param("weight_factor", weightingFactor_, 1.0);
  nodeHandle_.param("run_hind_leg_stance_detection", runHindLegStanceDetection_, true); // TODO: add to config file

  // For filter chain.
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

  //bool use_bag = true;

  // (New:) Foot tip position Subscriber for Foot tip - Elevation comparison
  // TESTED BY CHANGING IF SCOPES..


  //! Uncomment:
  //if(driftAdjustment_){
  //    if(!use_bag) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &SupportSurfaceEstimation::footTipStanceCallback, this);
  //    else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 1, &SupportSurfaceEstimation::footTipStanceCallback, this);
  //}

  // NEW: publish foot tip markers
  //footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000);
  //elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("map_bound_markers_rviz", 1000);
  //planeFitVisualizationPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("plane_fit_visualization_marker_list", 1000);
  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);
  supportSurfaceAddingAreaPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("adding_area", 1000);

  //ElevationMap map;

  //! uncomment!!
  //initializeFootTipMarkers();

  // NEW: Publish data, for parameter tuning and visualization
  //tuningPublisher1_ = nodeHandle_.advertise<elevation_mapping::PerformanceAssessment>("performance_assessment", 1000);

  // NEW: publish clored pointcloud visualizing the local pointcloud variance.
  //coloredPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("variance_pointcloud", 1);

  // Initializing some class variables.
  //heightDifferenceFromComparison_ = 0.0;
  //oldDiffComparisonUpdate_ = 0.0;
  //estimatedDrift_ = 0.0;
  //estimatedDriftChange_ = 0.0;
  //estimatedDriftChangeVariance_ = 0.0;
  //usedWeight_ = 0.0;
  //footTipOutsideBounds_ = true;
  //estimatedKalmanDiff_ = 0.0;
  //estimatedKalmanDiffIncrement_ = 0.0;
  //PEstimatedKalmanDiffIncrement_ = 0.0;
  //performanceAssessment_ = 0.0;
  //performanceAssessmentFlat_ = 0.0;
  //driftEstimationPID_ = 0.0;
  //highGrassMode_ = false;
  //isInStanceLeft_ = false;
  //isInStanceLeftHind_ = false;
  //isInStanceRight_ = false;
  //isInStanceRightHind_ = false;
  supportSurfaceInitializationTrigger_ = false;
  // END NEW
  std::cout << "Called the constructor of the SupportSurfaceEstimation!!" << std::endl;

  initialTime_ = ros::Time::now();
}

SupportSurfaceEstimation::~SupportSurfaceEstimation()
{
}

bool SupportSurfaceEstimation::updateSupportSurfaceEstimation(std::string tip){
    // Here all the functions are called and weighting is set..

    //penetrationDepthContinuityPropagation(); // Deprecated
    //terrainContinuityPropagation(); // Deprecated
    //penetrationDepthContinuityProcessing(tip); // Switched Off
    //terrainContinuityProcessing(); // Switched Off
    //footTipBasedElevationMapIncorporation(); // Switched Off
    gaussianProcessSmoothing(tip);
    return true;
}

bool SupportSurfaceEstimation::gaussianProcessSmoothing(std::string tip){

    // Uncertainty Estimation based on low pass filtered error between foot tip height and predicted support Surface.
    setSupportSurfaceUncertaintyEstimation(tip);

    // TODO: these params into config file.
    double tileResolution = 0.08;
    double tileDiameter = 0.23; // HAcked larger, if slow, change this here
    double sideLengthAddingPatch = 1.3;
    //setSmoothingTiles(tileResolution, tileDiameter, sideLengthAddingPatch, tip);

    return true;
}

bool SupportSurfaceEstimation::setSupportSurfaceUncertaintyEstimation(std::string tip){

    // TODO: try to access rawMap_.
    //ElevationMap map_;

    //Position pos(0.2,0.2);
    //rawMap_.atPosition("elevation", pos);

    //Eigen::Vector3f latestTip;
    //if (tip == "left") latestTip = getLatestLeftStance();
    //if (tip == "right") latestTip = getLatestRightStance();

    // So far not low pass filtered!! -> create Vector for that (TODO)
    //Position latestTipPosition(latestTip(0), latestTip(1));
    //double diff = fabs(latestTip(2) - supportMapGP_.atPosition("elevation_gp_added", latestTipPosition));
    //supportSurfaceUncertaintyEstimation_ = diff;
    //if (!isnan(diff)) cumulativeSupportSurfaceUncertaintyEstimation_ += diff;
    return true;
}

} /* namespace */
