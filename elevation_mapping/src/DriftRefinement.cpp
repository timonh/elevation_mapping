/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/DriftRefinement.hpp"

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

DriftRefinement::DriftRefinement(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{

  std::cout << "called the drift refinement Constructor" << std::endl;
  elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);

  // Launching parameters.
  nodeHandle_.param("run_drift_adjustment", driftAdjustment_, true);
  nodeHandle_.param("apply_frame_correction", applyFrameCorrection_, true);
  nodeHandle_.param("kp", kp_, 0.24);
  nodeHandle_.param("ki", ki_, 0.67);
  nodeHandle_.param("kd", kd_, -0.07);
  nodeHandle_.param("weight_factor", weightingFactor_, 1.0);
  nodeHandle_.param("run_hind_leg_stance_detection", runHindLegStanceDetection_, true); // TODO: add to config file

  //! Uncomment:
  //if(driftAdjustment_){
  //    if(!use_bag) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &DriftRefinement::footTipStanceCallback, this);
  //    else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 1, &DriftRefinement::footTipStanceCallback, this);
  //}

  // NEW: publish foot tip markers
  //footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000); // STANCE PROCESSOR
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("map_bound_markers_rviz", 1000);
  //planeFitVisualizationPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("plane_fit_visualization_marker_list", 1000);
  //varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);
  //supportSurfaceAddingAreaPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("adding_area", 1000);

  // NEW: Publish data, for parameter tuning and visualization
  //tuningPublisher1_ = nodeHandle_.advertise<elevation_mapping::PerformanceAssessment>("performance_assessment", 1000);

  // NEW: publish clored pointcloud visualizing the local pointcloud variance.
  coloredPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("variance_pointcloud", 1);

  // Initializing some class variables.
  heightDifferenceFromComparison_ = 0.0;
  oldDiffComparisonUpdate_ = 0.0;
  estimatedDrift_ = 0.0;
  estimatedDriftChange_ = 0.0;
  estimatedDriftChangeVariance_ = 0.0;
  usedWeight_ = 0.0;
  footTipOutsideBounds_ = true;
  estimatedKalmanDiff_ = 0.0;
  estimatedKalmanDiffIncrement_ = 0.0;
  PEstimatedKalmanDiffIncrement_ = 0.0;
  performanceAssessment_ = 0.0;
  performanceAssessmentFlat_ = 0.0;
  driftEstimationPID_ = 0.0;
  highGrassMode_ = false;
  isInStanceLeft_ = false;
  isInStanceLeftHind_ = false;
  isInStanceRight_ = false;
  isInStanceRightHind_ = false;
  supportSurfaceInitializationTrigger_ = false;
  // END NEW

  std::cout << "called the drift refinement Constructor" << std::endl;

  initialTime_ = ros::Time::now();
}

DriftRefinement::~DriftRefinement()
{
}

} /* namespace */
