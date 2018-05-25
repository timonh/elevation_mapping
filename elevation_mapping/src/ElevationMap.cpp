/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

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

ElevationMap::ElevationMap(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy",
              "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan",
              "sensor_z_at_lowest_scan", "foot_tip_elevation", "support_surface", "elevation_inpainted", "elevation_smooth", "vegetation_height", "vegetation_height_smooth",
              "support_surface", "support_surface_smooth", "support_surface_added"}),//, "support_surface_smooth_inpainted", "support_surface_added"}),
      fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      supportMap_({"elevation", "elevation_inpainted", "elevation_smooth"}), // New
      hasUnderlyingMap_(false),
      visibilityCleanupDuration_(0.0),
      filterChain_("grid_map::GridMap"), // New
      filterChain2_("grid_map::GridMap")
{
    // Timon added foot_tip_elevation layer
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  supportMap_.setBasicLayers({"elevation", "elevation_inpainted", "elevation_smooth"}); // New




  clear();

  // TEST:
  elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);
  elevationMapSupportPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_elevation", 1);
  elevationMapInpaintedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface", 1);
  // END TEST

  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
      nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
  // TODO if (enableVisibilityCleanup_) when parameter cleanup is ready.
  visbilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

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

  bool use_bag = true;

  // (New:) Foot tip position Subscriber for Foot tip - Elevation comparison
  // TESTED BY CHANGING IF SCOPES..
  if(driftAdjustment_){
      if(!use_bag) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &ElevationMap::footTipStanceCallback, this);
      else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 1, &ElevationMap::footTipStanceCallback, this);
  }

  // NEW: publish foot tip markers
  footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000);
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("map_bound_markers_rviz", 1000);
  planeFitVisualizationPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("plane_fit_visualization_marker_list", 1000);
  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("variances", 1000);
  supportSurfaceAddingAreaPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("adding_area", 1000);
  initializeFootTipMarkers();

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

  initialTime_ = ros::Time::now();
}

ElevationMap::~ElevationMap()
{
}

void ElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position)
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  fusedMap_.setGeometry(length, resolution, position);
  supportMap_.setGeometry(length, resolution, position); // New
  ROS_INFO_STREAM("Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and "  << rawMap_.getSize()(1) << " columns.");
}

bool ElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, Eigen::VectorXf& spatialVariances, const ros::Time& timestamp, const Eigen::Affine3d& transformationSensorToMap)
{

  if (pointCloud->size() != pointCloudVariances.size()) {
    ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
              (int) pointCloud->size(), (int) pointCloudVariances.size());
    return false;
  }

  if(false) publishSpatialVariancePointCloud(pointCloud, spatialVariances);
  // TESTING:
  //std::cout << "TIMING OF ADDING!! " << std::endl;
  // END TESTING


  // Initialization for time calculation.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  //! TEST about the two threads
//  std::thread::id this_id = std::this_thread::get_id();
//  std::cout << "This is the thread ADD: " << this_id << std::endl;
  //! END TEST


  // Update initial time if it is not initialized.
  if (initialTime_.toSec() == 0) {
    initialTime_ = timestamp;
  }
  const double scanTimeSinceInitialization = (timestamp - initialTime_).toSec();

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    auto& point = pointCloud->points[i];
    Index index;
    Position position(point.x, point.y);
    if (!rawMap_.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

    auto& elevation = rawMap_.at("elevation", index);
    auto& variance = rawMap_.at("variance", index);
    auto& horizontalVarianceX = rawMap_.at("horizontal_variance_x", index);
    auto& horizontalVarianceY = rawMap_.at("horizontal_variance_y", index);
    auto& horizontalVarianceXY = rawMap_.at("horizontal_variance_xy", index);
    auto& color = rawMap_.at("color", index);
    auto& time = rawMap_.at("time", index);
    auto& lowestScanPoint = rawMap_.at("lowest_scan_point", index);
    auto& sensorXatLowestScan = rawMap_.at("sensor_x_at_lowest_scan", index);
    auto& sensorYatLowestScan = rawMap_.at("sensor_y_at_lowest_scan", index);
    auto& sensorZatLowestScan = rawMap_.at("sensor_z_at_lowest_scan", index);

    auto& footTipElevation = rawMap_.at("foot_tip_elevation", index); // New
    auto& elevationInpainted = rawMap_.at("elevation_inpainted", index); // New
    auto& elevationSmooth = rawMap_.at("elevation_smooth", index); // New
    auto& elevationSupport = supportMap_.at("elevation", index); // New
    auto& elevationInpaintedSupport = supportMap_.at("elevation_inpainted", index); // New
    auto& elevationSmoothSupport = supportMap_.at("elevation_smooth", index); // New
    auto& vegetationHeight = rawMap_.at("vegetation_height", index); // New
    auto& vegetationHeightSmooth = rawMap_.at("vegetation_height_smooth", index); // New
    auto& supportSurface = rawMap_.at("support_surface", index); // New
    auto& supportSurfaceSmooth = rawMap_.at("support_surface_smooth", index); // New
    auto& supportSurfaceAdded = rawMap_.at("support_surface_added", index);


    const float& pointVariance = pointCloudVariances(i);
    const float scanTimeSinceInitialization = (timestamp - initialTime_).toSec();

    if (!rawMap_.isValid(index)) {
      // No prior information in elevation map, use measurement.
      elevation = point.z;
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
      colorVectorToValue(point.getRGBVector3i(), color);


      // TODO: to assign point.z to these may be not sound, as point.z comes from the pointcloud..
      footTipElevation = 0.0;
      elevationInpainted = point.z;
      elevationSmooth = point.z;
      elevationSupport = point.z;
      elevationInpaintedSupport = point.z;
      elevationSmoothSupport = point.z;
      supportSurface = point.z;
      supportSurfaceSmooth = point.z; // Hacked, testing what happens to the support surface movements in high grass
      vegetationHeight = point.z;
      vegetationHeightSmooth = point.z;
      supportSurfaceAdded = point.z;

      continue;
    }

    // Deal with multiple heights in one cell.
    const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance);
    if (mahalanobisDistance > mahalanobisDistanceThreshold_) {
      if (scanTimeSinceInitialization - time <= scanningDuration_ && elevation > point.z) {
        // Ignore point if measurement is from the same point cloud (time comparison) and
        // if measurement is lower then the elevation in the map.
      } else if (scanTimeSinceInitialization - time <= scanningDuration_) {
        // If point is higher.
        elevation = point.z;
        variance = pointVariance;
      } else {
        variance += multiHeightNoise_;
      }
      continue;
    }

    // Store lowest points from scan for visibility checking.
    const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
    if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint){
      lowestScanPoint = pointHeightPlusUncertainty;
      const Position3 sensorTranslation(transformationSensorToMap.translation());
      sensorXatLowestScan = sensorTranslation.x();
      sensorYatLowestScan = sensorTranslation.y();
      sensorZatLowestScan = sensorTranslation.z();
    }

    // Fuse measurement with elevation map data.
    elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
    variance = (pointVariance * variance) / (pointVariance + variance);
    // TODO Add color fusion.
    colorVectorToValue(point.getRGBVector3i(), color);
    time = scanTimeSinceInitialization;

    // Horizontal variances are reset.
    horizontalVarianceX = minHorizontalVariance_;
    horizontalVarianceY = minHorizontalVariance_;
    horizontalVarianceXY = 0.0;
  }

  // DEBUG
  //std::cout << "THATS WHERE I GOT BEFORE STOPPING!!1 \n";

  clean();
  rawMap_.setTimestamp(timestamp.toNSec()); // Point cloud stores time in microseconds.

  const ros::WallDuration duration = ros::WallTime::now() - methodStartTime;
  ROS_INFO("Raw map has been updated with a new point cloud in %f s.", duration.toSec());

  // DEBUG
  //std::cout << "THATS WHERE I GOT BEFORE STOPPING!!2 \n";
  return true;
}

bool ElevationMap::update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
                          const grid_map::Matrix& horizontalVarianceUpdateY,
                          const grid_map::Matrix& horizontalVarianceUpdateXY, const ros::Time& time)
{
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

  const auto& size = rawMap_.getSize();

  if (!((Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all()
      && (Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all()
      && (Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all()
      && (Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
    ROS_ERROR("The size of the update matrices does not match.");
    return false;
  }

  rawMap_.get("variance") += varianceUpdate;
  rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
  rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
  rawMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
  clean();
  rawMap_.setTimestamp(time.toNSec());

  return true;
}

bool ElevationMap::fuseAll()
{
  ROS_DEBUG("Requested to fuse entire elevation map.");
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fuse(Index(0, 0), fusedMap_.getSize());
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length)
{
  ROS_DEBUG("Requested to fuse an area of the elevation map with center at (%f, %f) and side lengths (%f, %f)",
            position[0], position[1], length[0], length[1]);

  Index topLeftIndex;
  Index submapBufferSize;

  // These parameters are not used in this function.
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_); // Hacked from fused to raw

  //! TEST about the two threads
  //std::thread::id this_id = std::this_thread::get_id();
  //std::cout << "This is the thread FUSE_AREA!!! : " << this_id << std::endl;
  //! END TEST

  // DEBUG
  //std::cout << "THATS WHERE I GOT BEFORE STOPPING!! 3 \n";

  getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength,
                       requestedIndexInSubmap, position, length, rawMap_.getLength(),
                       rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getSize(),
                       rawMap_.getStartIndex());

  // DEBUG
  //std::cout << "TIMING STUDY 1" << std::endl;
  //scopedLock.unlock(); // HAcked!!!
  //std::cout << "TIMING STUDY 2" << std::endl;
  // END DEBUG

  return fuse(topLeftIndex, submapBufferSize);
}

bool ElevationMap::fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size)
{
  ROS_DEBUG("Fusing elevation map...");

  // Nothing to do.
  if ((size == 0).any()) return false;

  // Initializations.
  const ros::WallTime methodStartTime(ros::WallTime::now());

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_); // HACKEDs

  //! ATTENTION!!! REMOVED THIS SCOPED LOCK!! CHECK WHAT THE CONSEQUENCE IS..
  // Copy raw elevation map data for safe multi-threading.
  //boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  //! TEST about the two threads
//  std::thread::id this_id = std::this_thread::get_id();
//  std::cout << "This is the thread FUSE: " << this_id << std::endl;
  //! END TEST

  auto& rawMapCopy = rawMap_;
  //scopedLockForRawData.unlock();

  // More initializations.
  const double halfResolution = fusedMap_.getResolution() / 2.0;
  const float minimalWeight = std::numeric_limits<float>::epsilon() * (float) 2.0;
  // Conservative cell inclusion for ellipse iterator.
  const double ellipseExtension = M_SQRT2 * fusedMap_.getResolution();

  // Check if there is the need to reset out-dated data.
  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) resetFusedData();

  // Align fused map with raw map.
  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) fusedMap_.move(rawMapCopy.getPosition());

  // For each cell in requested area.
  for (SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {

    // Check if fusion for this cell has already been done earlier.
    if (fusedMap_.isValid(*areaIterator)) continue;

    if (!rawMapCopy.isValid(*areaIterator)) {
      // This is an empty cell (hole in the map).
      // TODO.
      continue;
    }

    // Get size of error ellipse.
    const float& sigmaXsquare = rawMapCopy.at("horizontal_variance_x", *areaIterator);
    const float& sigmaYsquare = rawMapCopy.at("horizontal_variance_y", *areaIterator);
    const float& sigmaXYsquare = rawMapCopy.at("horizontal_variance_xy", *areaIterator);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << sigmaXsquare, sigmaXYsquare, sigmaXYsquare, sigmaYsquare;
    // 95.45% confidence ellipse which is 2.486-sigma for 2 dof problem.
    // http://www.reid.ai/2012/09/chi-squared-distribution-table-with.html
    const double uncertaintyFactor = 2.486; // sqrt(6.18)
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());

    Eigen::Array2d::Index maxEigenvalueIndex;
    eigenvalues.maxCoeff(&maxEigenvalueIndex);
    Eigen::Array2d::Index minEigenvalueIndex;
    maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
    const Length ellipseLength =  2.0 * uncertaintyFactor * Length(eigenvalues(maxEigenvalueIndex), eigenvalues(minEigenvalueIndex)).sqrt() + ellipseExtension;
    const double ellipseRotation(atan2(solver.eigenvectors().col(maxEigenvalueIndex).real()(1), solver.eigenvectors().col(maxEigenvalueIndex).real()(0)));

    // Requested length and position (center) of submap in map.
    Position requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);
    EllipseIterator ellipseIterator(rawMapCopy, requestedSubmapPosition, ellipseLength, ellipseRotation);

    // Prepare data fusion.
    Eigen::ArrayXf means, weights;
    const unsigned int maxNumberOfCellsToFuse = ellipseIterator.getSubmapSize().prod();
    means.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);
    WeightedEmpiricalCumulativeDistributionFunction<float> lowerBoundDistribution;
    WeightedEmpiricalCumulativeDistributionFunction<float> upperBoundDistribution;

    float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
    float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
    Eigen::Rotation2Dd rotationMatrix(ellipseRotation);
    std::string maxEigenvalueLayer, minEigenvalueLayer;
    if (maxEigenvalueIndex == 0) {
      maxEigenvalueLayer = "horizontal_variance_x";
      minEigenvalueLayer = "horizontal_variance_y";
    } else {
      maxEigenvalueLayer = "horizontal_variance_y";
      minEigenvalueLayer = "horizontal_variance_x";
    }

    // For each cell in error ellipse.
    size_t i = 0;
    for (; !ellipseIterator.isPastEnd(); ++ellipseIterator) {
      if (!rawMapCopy.isValid(*ellipseIterator)) {
        // Empty cell in submap (cannot be center cell because we checked above).
        continue;
      }

      means[i] = rawMapCopy.at("elevation", *ellipseIterator);

      // Compute weight from probability.
      Position absolutePosition;
      rawMapCopy.getPosition(*ellipseIterator, absolutePosition);
      Eigen::Vector2d distanceToCenter = (rotationMatrix * (absolutePosition - requestedSubmapPosition)).cwiseAbs();

      float probability1 =
            cumulativeDistributionFunction(distanceToCenter.x() + halfResolution, 0.0, maxStandardDeviation)
          - cumulativeDistributionFunction(distanceToCenter.x() - halfResolution, 0.0, maxStandardDeviation);
      float probability2 =
            cumulativeDistributionFunction(distanceToCenter.y() + halfResolution, 0.0, minStandardDeviation)
          - cumulativeDistributionFunction(distanceToCenter.y() - halfResolution, 0.0, minStandardDeviation);

      const float weight = max(minimalWeight, probability1 * probability2);
      weights[i] = weight;
      const float standardDeviation = sqrt(rawMapCopy.at("variance", *ellipseIterator));
      lowerBoundDistribution.add(means[i] - 2.0 * standardDeviation, weight);
      upperBoundDistribution.add(means[i] + 2.0 * standardDeviation, weight);

      i++;
    }

    if (i == 0) {
      // Nothing to fuse.
      fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
      fusedMap_.at("lower_bound", *areaIterator) = rawMapCopy.at("elevation", *areaIterator) - 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("upper_bound", *areaIterator) = rawMapCopy.at("elevation", *areaIterator) + 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
      continue;
    }

    // Fuse.
    means.conservativeResize(i);
    weights.conservativeResize(i);

    float mean = (weights * means).sum() / weights.sum();

    if (!std::isfinite(mean)) {
      ROS_ERROR("Something went wrong when fusing the map: Mean = %f", mean);
      continue;
    }

    // Add to fused map.
    fusedMap_.at("elevation", *areaIterator) = mean;
    lowerBoundDistribution.compute();
    upperBoundDistribution.compute();
    fusedMap_.at("lower_bound", *areaIterator) = lowerBoundDistribution.quantile(0.01); // TODO
    fusedMap_.at("upper_bound", *areaIterator) = upperBoundDistribution.quantile(0.99); // TODO
    // TODO Add fusion of colors.
    fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
  }

  fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

  const ros::WallDuration duration(ros::WallTime::now() - methodStartTime);
  ROS_INFO("Elevation map has been fused in %f s.", duration.toSec());

  return true;
}

bool ElevationMap::clear()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
  rawMap_.clearAll();
  rawMap_.resetTimestamp();
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
  visibilityCleanupMap_.clearAll();
  visibilityCleanupMap_.resetTimestamp();
  return true;
}

void ElevationMap::visibilityCleanup(const ros::Time& updatedTime)
{
  // Get current time to compute calculation time.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  const double timeSinceInitialization = (updatedTime - initialTime_).toSec();

  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  visibilityCleanupMap_ = rawMap_;
  rawMap_.clear("lowest_scan_point");
  rawMap_.clear("sensor_x_at_lowest_scan");
  rawMap_.clear("sensor_y_at_lowest_scan");
  rawMap_.clear("sensor_z_at_lowest_scan");
  scopedLockForRawData.unlock();
  visibilityCleanupMap_.add("max_height");

  // Create max. height layer with ray tracing.
  for (GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) continue;
    const auto& lowestScanPoint = visibilityCleanupMap_.at("lowest_scan_point", *iterator);
    const auto& sensorXatLowestScan = visibilityCleanupMap_.at("sensor_x_at_lowest_scan", *iterator);
    const auto& sensorYatLowestScan = visibilityCleanupMap_.at("sensor_y_at_lowest_scan", *iterator);
    const auto& sensorZatLowestScan = visibilityCleanupMap_.at("sensor_z_at_lowest_scan", *iterator);
    if (std::isnan(lowestScanPoint)) continue;
    Index indexAtSensor;
    if(!visibilityCleanupMap_.getIndex(Position(sensorXatLowestScan, sensorYatLowestScan), indexAtSensor)) continue;
    Position point;
    visibilityCleanupMap_.getPosition(*iterator, point);
    float pointDiffX = point.x() - sensorXatLowestScan;
    float pointDiffY = point.y() - sensorYatLowestScan;
    float distanceToPoint = sqrt(pointDiffX * pointDiffX + pointDiffY * pointDiffY);
    if (distanceToPoint > 0.0) {
      for (grid_map::LineIterator iterator(visibilityCleanupMap_, indexAtSensor, *iterator); !iterator.isPastEnd(); ++iterator) {
        Position cellPosition;
        visibilityCleanupMap_.getPosition(*iterator, cellPosition);
        const float cellDiffX = cellPosition.x() - sensorXatLowestScan;
        const float cellDiffY = cellPosition.y() - sensorYatLowestScan;
        const float distanceToCell = distanceToPoint - sqrt(cellDiffX * cellDiffX + cellDiffY * cellDiffY);
        const float maxHeightPoint = lowestScanPoint + (sensorZatLowestScan - lowestScanPoint) / distanceToPoint * distanceToCell;
        auto& cellMaxHeight = visibilityCleanupMap_.at("max_height", *iterator);
        if (std::isnan(cellMaxHeight) || cellMaxHeight > maxHeightPoint) {
          cellMaxHeight = maxHeightPoint;
        }
      }
    }
  }

  // Vector of indices that will be removed.
  std::vector<Position> cellPositionsToRemove;
  for (GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) continue;
    const auto& time = visibilityCleanupMap_.at("time", *iterator);
    if (timeSinceInitialization - time > scanningDuration_) {
      // Only remove cells that have not been updated during the last scan duration.
      // This prevents a.o. removal of overhanging objects.
      const auto& elevation = visibilityCleanupMap_.at("elevation", *iterator);
      const auto& variance = visibilityCleanupMap_.at("variance", *iterator);
      const auto& maxHeight = visibilityCleanupMap_.at("max_height", *iterator);
      if (!std::isnan(maxHeight) && elevation - 3.0 * sqrt(variance) > maxHeight) {
        Position position;
        visibilityCleanupMap_.getPosition(*iterator, position);
        cellPositionsToRemove.push_back(position);
      }
    }
  }

  // Remove points in current raw map.
  scopedLockForRawData.lock();
  for(const auto& cellPosition : cellPositionsToRemove){
    Index index;
    if (!rawMap_.getIndex(cellPosition, index)) continue;
    if(rawMap_.isValid(index)){
      rawMap_.at("elevation", index) = NAN;
    }
  }
  scopedLockForRawData.unlock();

  // Publish visibility cleanup map for debugging.
  publishVisibilityCleanupMap();

  ros::WallDuration duration(ros::WallTime::now() - methodStartTime);
  ROS_INFO("Visibility cleanup has been performed in %f s (%d points).", duration.toSec(), (int)cellPositionsToRemove.size());
  if(duration.toSec() > visibilityCleanupDuration_)
    ROS_WARN("Visibility cleanup duration is too high (current rate is %f).", 1.0 / duration.toSec());
}

void ElevationMap::move(const Eigen::Vector2d& position)
{

  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_); //HACKED
  std::vector<BufferRegion> newRegions;

  if (rawMap_.move(position, newRegions)) {
    ROS_DEBUG("Elevation map has been moved to position (%f, %f).", rawMap_.getPosition().x(), rawMap_.getPosition().y());
    if (hasUnderlyingMap_) rawMap_.addDataFrom(underlyingMap_, false, false, true);
  }
}

bool ElevationMap::publishRawElevationMap()
{
  if (!hasRawMapSubscribers()) return false;
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
  grid_map::GridMap rawMapCopy = rawMap_;
  scopedLock.unlock();
  rawMapCopy.erase("lowest_scan_point");
  rawMapCopy.erase("sensor_x_at_lowest_scan");
  rawMapCopy.erase("sensor_y_at_lowest_scan");
  rawMapCopy.erase("sensor_z_at_lowest_scan");
  rawMapCopy.add("standard_deviation", rawMapCopy.get("variance").array().sqrt().matrix());
  rawMapCopy.add("horizontal_standard_deviation", (rawMapCopy.get("horizontal_variance_x") + rawMapCopy.get("horizontal_variance_y")).array().sqrt().matrix());
  rawMapCopy.add("two_sigma_bound", rawMapCopy.get("elevation") + 2.0 * rawMapCopy.get("variance").array().sqrt().matrix());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(rawMapCopy, message);
  elevationMapRawPublisher_.publish(message);
  ROS_DEBUG("Elevation map raw has been published.");
  return true;
}

bool ElevationMap::publishFusedElevationMap()
{
  if (!hasFusedMapSubscribers()) return false;
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  GridMap fusedMapCopy = fusedMap_;
  scopedLock.unlock();
  fusedMapCopy.add("uncertainty_range", fusedMapCopy.get("upper_bound") - fusedMapCopy.get("lower_bound"));
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(fusedMapCopy, message);
  elevationMapFusedPublisher_.publish(message);
  ROS_DEBUG("Elevation map (fused) has been published.");
  return true;
}

bool ElevationMap::publishVisibilityCleanupMap()
{
  if (visbilityCleanupMapPublisher_.getNumSubscribers() < 1) return false;
  boost::recursive_mutex::scoped_lock scopedLock(visibilityCleanupMapMutex_);
  grid_map::GridMap visibilityCleanupMapCopy = visibilityCleanupMap_;
  scopedLock.unlock();
  visibilityCleanupMapCopy.erase("elevation");
  visibilityCleanupMapCopy.erase("variance");
  visibilityCleanupMapCopy.erase("horizontal_variance_x");
  visibilityCleanupMapCopy.erase("horizontal_variance_y");
  visibilityCleanupMapCopy.erase("horizontal_variance_xy");
  visibilityCleanupMapCopy.erase("color");
  visibilityCleanupMapCopy.erase("time");
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(visibilityCleanupMapCopy, message);
  visbilityCleanupMapPublisher_.publish(message);
  ROS_DEBUG("Visibility cleanup map has been published.");
  return true;
}

bool ElevationMap::publishSpatialVariancePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,Eigen::VectorXf& spatialVariances)
{
  //std::cout << "cols: " << spatialVariances.cols() << std::endl;
  //std::cout << "rows: " << spatialVariances.rows() << std::endl;

  //if(spatialVariances.rows() == pointCloud->size()) std::cout << "GREAT SUCCESS< THUMBS UP!!" << std::endl;
  //else std::cout << (spatialVariances.rows() - pointCloud->size()) << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> pointCloudColored;
  sensor_msgs::PointCloud2 variancePointCloud;
  for(unsigned int i = 0; i < pointCloud->size(); ++i){
      pcl::PointXYZRGB pointnew = pointCloud->points[i];
      double factor = 3.5 * pow(10,6);
      pointnew.r = (int)min(spatialVariances(i,0) * factor, 255.0);
      pointnew.g = 0;
      pointnew.b = 0;
      pointnew.a = 1;
      pointCloudColored.push_back(pointnew);
  }

  //pcl::PointCloud<pcl::PointXYZRGB> pointCloudColoredTransformed;
  //pointCloudColored.header.frame_id = "odom";
  //pcl_ros::transformPointCloud("odom_drift_adjusted", pointCloudColored,
  //                             pointCloudColoredTransformed, odomDriftAdjustedTransformListener_);

  pcl::toROSMsg(pointCloudColored, variancePointCloud);
  variancePointCloud.header.frame_id = "odom_drift_adjusted"; // Check if needing transform!!
  bool publishVariancePointCloud = true;
  if(publishVariancePointCloud) coloredPointCloudPublisher_.publish(variancePointCloud);

  return true;
}

grid_map::GridMap& ElevationMap::getRawGridMap()
{
  return rawMap_;
}

grid_map::GridMap& ElevationMap::getFusedGridMap()
{
  return fusedMap_;
}

ros::Time ElevationMap::getTimeOfLastUpdate()
{
  return ros::Time().fromNSec(rawMap_.getTimestamp());
}

ros::Time ElevationMap::getTimeOfLastFusion()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return ros::Time().fromNSec(fusedMap_.getTimestamp());
}

const kindr::HomTransformQuatD& ElevationMap::getPose()
{
  return pose_;
}

bool ElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position)
{
  kindr::Position3D positionInGridFrame;
  if (!rawMap_.getPosition3("elevation", index, positionInGridFrame.vector())) return false;
  position = pose_.transform(positionInGridFrame);
  return true;
}

boost::recursive_mutex& ElevationMap::getFusedDataMutex()
{
  return fusedMapMutex_;
}

boost::recursive_mutex& ElevationMap::getRawDataMutex()
{
  return rawMapMutex_;
}

bool ElevationMap::clean()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.get("variance") = rawMap_.get("variance").unaryExpr(VarianceClampOperator<float>(minVariance_, maxVariance_));
  rawMap_.get("horizontal_variance_x") = rawMap_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  rawMap_.get("horizontal_variance_y") = rawMap_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData()
{
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
}

void ElevationMap::setFrameId(const std::string& frameId)
{
  rawMap_.setFrameId(frameId);
  fusedMap_.setFrameId(frameId);
}

const std::string& ElevationMap::getFrameId()
{
  return rawMap_.getFrameId();
}

bool ElevationMap::hasRawMapSubscribers() const
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

bool ElevationMap::hasFusedMapSubscribers() const
{
  if (elevationMapFusedPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

void ElevationMap::underlyingMapCallback(const grid_map_msgs::GridMap& underlyingMap)
{

  ROS_INFO("Updating underlying map.");
  GridMapRosConverter::fromMessage(underlyingMap, underlyingMap_);
  if (underlyingMap_.getFrameId() != rawMap_.getFrameId()) {
    ROS_ERROR_STREAM("The underlying map does not have the same map frame ('" <<underlyingMap_.getFrameId()
                     << "') as the elevation map ('" << rawMap_.getFrameId() << "').");
    return;
  }
  if (!underlyingMap_.exists("elevation")) {
    ROS_ERROR_STREAM("The underlying map does not have an 'elevation' layer.");
    return;
  }
  if (!underlyingMap_.exists("variance")) underlyingMap_.add("variance", minVariance_);
  if (!underlyingMap_.exists("horizontal_variance_x")) underlyingMap_.add("horizontal_variance_x", minHorizontalVariance_);
  if (!underlyingMap_.exists("horizontal_variance_y")) underlyingMap_.add("horizontal_variance_y", minHorizontalVariance_);
  if (!underlyingMap_.exists("color")) underlyingMap_.add("color", 0.0);
  underlyingMap_.setBasicLayers(rawMap_.getBasicLayers());
  hasUnderlyingMap_ = true;
  rawMap_.addDataFrom(underlyingMap_, false, false, true);
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

void ElevationMap::footTipStanceCallback(const quadruped_msgs::QuadrupedState& quadrupedState)
{

  //std::cout << "Calling Back still!!!" << std::endl;

  //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);
  // Set class variables.
  LFTipPosition_(0) = (double)quadrupedState.contacts[0].position.x;
  LFTipPosition_(1) = (double)quadrupedState.contacts[0].position.y;
  LFTipPosition_(2) = (double)quadrupedState.contacts[0].position.z;
  RFTipPosition_(0) = (double)quadrupedState.contacts[1].position.x;
  RFTipPosition_(1) = (double)quadrupedState.contacts[1].position.y;
  RFTipPosition_(2) = (double)quadrupedState.contacts[1].position.z;
  LFTipState_ = quadrupedState.contacts[0].state;
  RFTipState_ = quadrupedState.contacts[1].state;


  if(runHindLegStanceDetection_){
      // Add hind legs for proprioceptive variance estimation.
      LHTipPosition_(0) = (double)quadrupedState.contacts[2].position.x;
      LHTipPosition_(1) = (double)quadrupedState.contacts[2].position.y;
      LHTipPosition_(2) = (double)quadrupedState.contacts[2].position.z;
      RHTipPosition_(0) = (double)quadrupedState.contacts[3].position.x;
      RHTipPosition_(1) = (double)quadrupedState.contacts[3].position.y;
      RHTipPosition_(2) = (double)quadrupedState.contacts[3].position.z;
      LHTipState_ = quadrupedState.contacts[2].state;
      RHTipState_ = quadrupedState.contacts[3].state;
  }


  // Get footprint position and orientation.
  setFootprint(quadrupedState.frame_transforms[3].transform);


  // Check if walking forward or backwards. TODO!
  //(double)quadrupedState.twist.twist.linear.x;

  // Detect start and end of stances for each of the two front foot tips.
  detectStancePhase();
  //detectStancePhase("right");
  frameCorrection();



}

bool ElevationMap::detectStancePhase()
{

    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);

    //! TEST about the two threads
    //std::thread::id this_id = std::this_thread::get_id();
    //std::cout << "This is the thread: " << this_id << std::endl;
    //! END TEST



    // Collect State Data for stance phase detection
    processStanceTriggerLeft_.push_back(LFTipState_);
    processStanceTriggerRight_.push_back(RFTipState_);

    // Constrain the size of the state arrays.
    if(processStanceTriggerLeft_.size() > 2000){
        processStanceTriggerLeft_.erase(processStanceTriggerLeft_.begin());
    }
    if(processStanceTriggerRight_.size() > 2000){
        processStanceTriggerRight_.erase(processStanceTriggerRight_.begin());
    }

    // Collect the foot tip position data (if foot tip in contact). (Prevent nans)
    if(LFTipState_ && isInStanceLeft_){
        LFTipStance_.push_back(LFTipPosition_);
    }
    if(RFTipState_ && isInStanceRight_){
        RFTipStance_.push_back(RFTipPosition_);
    }

    // Stance Detection
    templateMatchingForStanceDetection("left", processStanceTriggerLeft_);
    templateMatchingForStanceDetection("right", processStanceTriggerRight_);

    // Here again for hind legs..
    if (runHindLegStanceDetection_){


        // Collect State Data for stance phase detection
        processStanceTriggerLeftHind_.push_back(LHTipState_);
        processStanceTriggerRightHind_.push_back(RHTipState_);

        // Constrain the size of the state arrays.
        if(processStanceTriggerLeftHind_.size() > 2000){
            processStanceTriggerLeftHind_.erase(processStanceTriggerLeftHind_.begin());
        }
        if(processStanceTriggerRightHind_.size() > 2000){
            processStanceTriggerRightHind_.erase(processStanceTriggerRightHind_.begin());
        }

        // Collect the foot tip position data (if foot tip in contact). (Prevent nans)
        if(LHTipState_ && isInStanceLeftHind_){
            LHTipStance_.push_back(LHTipPosition_);
        }
        if(RHTipState_ && isInStanceRightHind_){
            RHTipStance_.push_back(RHTipPosition_);
        }

        // Stance Detection
        templateMatchingForStanceDetection("lefthind", processStanceTriggerLeftHind_);
        templateMatchingForStanceDetection("righthind", processStanceTriggerRightHind_);


    }

    return true;
}

bool ElevationMap::templateMatchingForStanceDetection(std::string tip, std::vector<bool> &stateVector)
{
    //std::cout << "tip: " << tip << std::endl;
    //std::cout << "statevec size: " << stateVector.size() << std::endl;
    //std::cout << isInStanceLeftHind_ << std::endl;


    // Recognition of the end of a stance phase of the front legs.
    if(stateVector.size() >= 16 && stateVector.end()[-1]+stateVector.end()[-2]+
            stateVector.end()[-3]+stateVector.end()[-4]+stateVector.end()[-5]+
            stateVector.end()[-6]+stateVector.end()[-7]+stateVector.end()[-8] >= 6 &&
            stateVector.end()[-9]+stateVector.end()[-10] +
            stateVector.end()[-11]+stateVector.end()[-12]+ stateVector.end()[-13]+
            stateVector.end()[-14]+stateVector.end()[-15] <= 6){
        if(tip == "left" && !isInStanceLeft_){
           // std::cout << "Start of LEFT stance" << std::endl;
            isInStanceLeft_ = 1;
        }
        if(tip == "lefthind" && !isInStanceLeftHind_){
            isInStanceLeftHind_ = 1;
        }
        if(tip == "right" && !isInStanceRight_){
           // std::cout << "Start of Right stance" << std::endl;
            isInStanceRight_ = 1;
        }
        if(tip == "righthind" && !isInStanceRightHind_){
            isInStanceRightHind_ = 1;
        }
    }

    // DEBUG:
    //std::cout << "tip: (OUTSIDE the state end detector) " << tip << std::endl;
    //std::cout << "stateVector.size: " << stateVector.size() << std::endl;
    //std::cout << "isInstancerighthind: " << isInStanceRightHind_ << std::endl;
    //std::cout << "processStanceTrigger: " << processStanceTriggerRightHind_.size() << std::endl;
    //std::cout << "processStanceTrigger: clssic " << processStanceTriggerRight_.size() << std::endl;


    // Recognition of the start of a stance phase of the front legs.
    if(stateVector.size() >= 16 &&
            stateVector.end()[-1] + stateVector.end()[-2] + stateVector.end()[-3]+
            stateVector.end()[-4]+stateVector.end()[-5] + stateVector.end()[-6]+
            stateVector.end()[-7]+stateVector.end()[-8] <= 3 &&
            stateVector.end()[-9]+stateVector.end()[-10] +
            stateVector.end()[-11]+stateVector.end()[-12]+ stateVector.end()[-13]+
            stateVector.end()[-14]+stateVector.end()[-15] >=1){

        if(tip == "left" && isInStanceLeft_){
            if(!processStance("left")) return false;
            isInStanceLeft_ = 0;
            processStanceTriggerLeft_.clear();
        }
        if(tip == "lefthind" && isInStanceLeftHind_){
            if(!processStance("lefthind")) return false;
            isInStanceLeftHind_ = 0;
            processStanceTriggerLeftHind_.clear();
        }
        if(tip == "right" && isInStanceRight_){
            if(!processStance("right")) return false;
            isInStanceRight_ = 0;
            processStanceTriggerRight_.clear();
        }
        if(tip == "righthind" && isInStanceRightHind_){
            if(!processStance("righthind")) return false;
            isInStanceRightHind_ = 0;
            processStanceTriggerRightHind_.clear();
        }
    }
    return true;
}

bool ElevationMap::processStance(std::string tip)
{
    //std::cout << "Processing: " << tip <<std::endl;

    // The ordering here is crucial!

    bool hind = false;
    // Delete the last 10 entries of the Foot Stance Position Vector, as these are used for transition detection
    deleteLastEntriesOfStances(tip);
    getAverageFootTipPositions(tip);

    bool runProprioceptiveRoughnessEstimation = true;
    if(runProprioceptiveRoughnessEstimation) proprioceptiveRoughnessEstimation(tip);


    if(tip != "lefthind" && tip != "righthind") footTipElevationMapComparison(tip);
    else hind = true;
    publishAveragedFootTipPositionMarkers(hind);


    // Performance Assessment, sensible if wanting to tune the system while walking on flat surface.
    //bool runPreformanceAssessmentForFlatGround = false;
    //if(runPreformanceAssessmentForFlatGround) performanceAssessmentMeanElevationMap();

    // Data Publisher for parameter tuning.
    //tuningPublisher1_.publish(performance_assessment_msg_); // HACKED FOR DEBUGGING!!



    return true;
}

bool ElevationMap::deleteLastEntriesOfStances(std::string tip)
{
    // Delete the last entries of the stance, as these might be in the false state
    // (half the number of elements used in template matching are deleted)
    if (tip == "left" && LFTipStance_.size() < 8){
        std::cout << "WARNING: LEFT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (tip == "right" && RFTipStance_.size() < 8){
        std::cout << "WARNING: RIGHT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (tip == "lefthind" && LHTipStance_.size() < 8){
        std::cout << "WARNING: LEFT HIND STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (tip == "righthind" && RHTipStance_.size() < 8){
        std::cout << "WARNING: RIGHT HIND STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else{
        for (unsigned int i = 0; i < 8; ++i){
            if (tip == "left"){
                LFTipStance_.pop_back();
            }
            if (tip == "right"){
                RFTipStance_.pop_back();
            }
            if (tip == "lefthind"){
                LHTipStance_.pop_back();
            }
            if (tip == "righthind"){
                RHTipStance_.pop_back();
            }
        }
    }
    return true;
}

bool ElevationMap::getAverageFootTipPositions(std::string tip)
{
    Eigen::Vector3f totalStance(0, 0, 0);
    std::vector<Eigen::Vector3f> stance;
    if(tip == "left"){
        stance = LFTipStance_;
        LFTipStance_.clear();
    }
    if(tip == "right"){
        stance = RFTipStance_;
        RFTipStance_.clear();
    }
    if(tip == "lefthind"){
        stance = LHTipStance_;
        LHTipStance_.clear();
    }
    if(tip == "righthind"){
        stance = RHTipStance_;
        RHTipStance_.clear();
    }


    // Derive mean foot tip positions
    if(stance.size() > 1){
        for (auto& n : stance){
            // Consider only those with state = 1  TODO!!!
            totalStance += n;
        }
        meanStance_ = totalStance / float(stance.size());
    }

    // Save the front mean tip positions for simple foot tip embedding into high grass detection
    if (tip == "left") frontLeftFootTip_ = meanStance_;
    if (tip == "right") frontRightFootTip_ = meanStance_;


    return true;
}

bool ElevationMap::publishAveragedFootTipPositionMarkers(bool hind)
{

    // TESTING:
   // std::cout << "xTip: " << meanStance_(0) << std::endl;
   // std::cout << "yTip: " << meanStance_(0) << std::endl;
    // END TESTING

    // Positions for publisher.
    geometry_msgs::Point p;
    p.x = meanStance_(0);
    p.y = meanStance_(1);
    p.z = meanStance_(2);

    // Coloring as function of applied weight.
    bool footTipColoring = true;
    double coloring_factor = 2.5;
    std_msgs::ColorRGBA c;
    c.g = 0;
    c.b = max(0.0, 1 - coloring_factor * usedWeight_); // TODO set after tuning to get sensible results..
    c.r = min(1.0, coloring_factor * usedWeight_);
    c.a = 0.5;

    if(hind){
        c.g = 1;
        c.b = 1;
        c.r = 1;
        c.a = 1;
    }

    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

    // If outside the area, where comparison can be made (i.e. no elevation map value is found) set black color.
    // Set yellow if foot tip is outside the bounds of the fused map.
    Position coloringPosition(p.x, p.y);
    if (isnan(rawMap_.atPosition("elevation", coloringPosition))){
        c.g = 0;
        c.b = 0;
        c.r = 0;
    }
    else if(footTipOutsideBounds_) c.g = 0.9;

    //scopedLock.unlock();

    // Check for nans
    if(p.x != p.x || p.y != p.y || p.z != p.z){
        std::cout << "NAN FOUND IN MEAN FOOT TIP POSITION!!" << std::endl;
    }
    else{
        footContactMarkerList_.points.push_back(p);
        if(footTipColoring)footContactMarkerList_.colors.push_back(c);
    }

    // Publish averaged foot tip positions
    footContactPublisher_.publish(footContactMarkerList_);


    // Uses the footContactMarkerList_, therefore called here.
    // Updates the map layer, that purely relies on the last n foot tips.
    //int numberOfConsideredFootTips = 4;

    //! HACKED AWAY, no foot tip layer updates now!!
    //updateFootTipBasedElevationMapLayer(numberOfConsideredFootTips);

    return true;
}

bool ElevationMap::publishFusedMapBoundMarkers(double& xTip, double& yTip,
                                               double& elevationFused, double& upperBoundFused, double& lowerBoundFused)
{
    geometry_msgs::Point p_elev, p_upper, p_lower;
    p_elev.x = p_upper.x = p_lower.x = xTip;
    p_elev.y = p_upper.y = p_lower.y = yTip;
    p_elev.z = elevationFused + heightDifferenceFromComparison_;
    p_upper.z = upperBoundFused + heightDifferenceFromComparison_;
    p_lower.z = lowerBoundFused + heightDifferenceFromComparison_;


    std_msgs::ColorRGBA c;
    c.r = 1;
    c.g = 0.5;
    c.b = 0;
    if(highGrassMode_){
        c.g = 0.3;
        c.r = 0.0;
        c.b = 0.0;
    }// New, colors to show if frame correction on or high grass detection..
    c.a = 1;
    //footContactMarkerList_.points.push_back(p_elev);
    //footContactMarkerList_.colors.push_back(c);

    elevationMapBoundMarkerList_.points.push_back(p_upper);
    elevationMapBoundMarkerList_.colors.push_back(c);

    elevationMapBoundMarkerList_.points.push_back(p_lower);
    elevationMapBoundMarkerList_.colors.push_back(c);

    elevationMapBoundPublisher_.publish(elevationMapBoundMarkerList_);

    return true;
}

bool ElevationMap::footTipElevationMapComparison(std::string tip)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceComparisonMutex_);
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);
    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);


    // New version
    double xTip, yTip, zTip = 0;
    xTip = meanStance_(0);
    yTip = meanStance_(1);
    zTip = meanStance_(2);


    // TODO: Clean nan supression scheme.
    bool useNewMethod = true;
    if(useNewMethod){

        // Offset of each step versus Elevation map.
        double verticalDifference = 0;
        // Weighted offset depending on the upper and lower bounds of the fused map.
        double weightedVerticalDifferenceIncrement = 0.0;

        // Horizontal position of the foot tip.
        Position tipPosition(xTip, yTip);

        // New here, check what isInside does..
        if(rawMap_.isInside(tipPosition)) updateSupportSurfaceEstimation(); // NEW !!!!!
        else std::cout << "FOOT TIP CONSIDERED NOT TO BE INSIDE!!!!! \n \n \n \n " << std::endl;

        // Make sure that the state is 1 and the foot tip is inside area covered by the elevation map.
        if(rawMap_.isInside(tipPosition) && !isnan(heightDifferenceFromComparison_)){ // HACKED FOR TESTS!!!
            float heightMapRaw = rawMap_.atPosition("elevation", tipPosition);
            float varianceMapRaw = rawMap_.atPosition("variance", tipPosition);
            float heightMapRawElevationCorrected = rawMap_.atPosition("elevation", tipPosition) + heightDifferenceFromComparison_; // Changed, since frame transformed grid map used.

            // Supress nans.
            if(!isnan(heightMapRaw)){

                // Calculate difference.
                verticalDifference = zTip - heightMapRaw;
                double verticalDifferenceCorrected = zTip - heightMapRawElevationCorrected;
               // std::cout << "HeightDifference CLASSICAL:    " << verticalDifference << "        HeightDifference CORRECTED:    " << verticalDifferenceCorrected << std::endl;

                // New Tuning piblisher.

                //performance_assessment_msg_.first_measure = verticalDifferenceCorrected;

                // Wait some stances to get sorted at the beginning.
                if(weightedDifferenceVector_.size() >= 4) performanceAssessment_ += fabs(verticalDifferenceCorrected);
                //std::cout << "Cumulative Performance Value (summed up weighted differences): " << performanceAssessment_ << std::endl;


                //performance_assessment_msg_.second_measure = performanceAssessment_;

                // Use 3 standard deviations of the uncertainty ellipse of the foot tip positions as fusion area.
                Eigen::Array2d ellipseAxes;
                ellipseAxes[0] = ellipseAxes[1] = std::max(6 * sqrt(rawMap_.atPosition("horizontal_variance_x",tipPosition)),
                                          6 * sqrt(rawMap_.atPosition("horizontal_variance_y",tipPosition)));

                // Get lower and upper bound of the fused map.
                auto boundTuple = getFusedCellBounds(tipPosition, ellipseAxes);
                double lowerBoundFused = std::get<0>(boundTuple);
                double elevationFused = std::get<1>(boundTuple);
                double upperBoundFused = std::get<2>(boundTuple);
               // std::cout << "lower: " << lowerBoundFused << " elev: " << elevationFused << " upper: " << upperBoundFused << std::endl;
                weightedVerticalDifferenceIncrement = gaussianWeightedDifferenceIncrement(lowerBoundFused, elevationFused, upperBoundFused, verticalDifference);

               // std::cout << "weightedVerticalDifferenceIncrement " << weightedVerticalDifferenceIncrement << std::endl;


                bool runPenetrationDepthVarianceEstimation = true;
                if (runPenetrationDepthVarianceEstimation) penetrationDepthVarianceEstimation(tip, verticalDifference);

                // DEBUG:
//                if (isnan(weightedVerticalDifferenceIncrement)){
//                    std::cout << "NAN IN WEIGHTED DIFFERENCE INCREMENT!!!!!!!" << std::endl;
//                    std::cout << "" << std::endl;
//                    std::cout << "" << std::endl;
//                    std::cout << "" << std::endl;
//                    weightedVerticalDifferenceIncrement = 0;
//                }
                // END DEBUG

                // TODO: Switch ON and OFF
                publishFusedMapBoundMarkers(xTip, yTip, elevationFused, upperBoundFused, lowerBoundFused);


                // TESTING DRIFT STUFF
//                auto driftTuple = filteredDriftEstimation(weightedVerticalDifferenceIncrement, estimatedDriftChange_, estimatedDriftChangeVariance_);
//                estimatedDriftChange_ = std::get<0>(driftTuple);
//                estimatedDriftChangeVariance_ = std::get<1>(driftTuple);
//                estimatedDrift_ += estimatedDriftChange_;
//                std::cout << "ESTIMATED DRIFT: " << estimatedDrift_ << std::endl;
//                std::cout << "ESTIMATED DRIFT Change: " << estimatedDriftChange_ << std::endl;
//                std::cout << "ESTIMATED DRIFT Change Variance: " << estimatedDriftChangeVariance_ << std::endl;

//                oldDiffComparisonUpdate_ = weightedVerticalDifferenceIncrement;
//                std::cout << "ESTIMATED oldDRIFT: " << oldDiffComparisonUpdate_ << std::endl;
                // END TESTING


                // TODO: test validity of these:

                // DEBUG
              //  std::cout << "heightDiff befor weighted difference vector creation: " << heightDifferenceFromComparison_ <<
              //               " weightedVerticalDiffIncrement: " << weightedVerticalDifferenceIncrement << std::endl;

                // Store the vertical difference history in a vector for PID controlling.
                weightedDifferenceVector_.push_back(heightDifferenceFromComparison_ + weightedVerticalDifferenceIncrement); // TODO: check viability
                if(weightedDifferenceVector_.size() > 6) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
             //   std::cout << "Weighted Height Difference: " << weightedDifferenceVector_[0] << "\n";

                // Longer weightedDifferenceVector for PID drift calculation
                if(!isnan(heightDifferenceFromComparison_)) PIDWeightedDifferenceVector_.push_back(heightDifferenceFromComparison_); // Removed the vertical difference increment, TEST IT!
                if(PIDWeightedDifferenceVector_.size() >= 30) PIDWeightedDifferenceVector_.erase(PIDWeightedDifferenceVector_.begin());


                // New: create one for the left and one for the right foot tips, to get separate drift estimation
                //if (tip == "right")

                // TODO: for Kalman drift estimation, and PID with left and right separated...

                // PID height offset calculation.
                double heightDiffPID = differenceCalculationUsingPID();

                // Avoid Old Nans to be included. TEST!! // TODO: check where nans might come from to avoid them earlier
//                if(!isnan(heightDiffPID))

//                // PID based drift estimation.
//                driftEstimationPID_ = driftCalculationUsingPID(tip);


//                if (isnan(driftEstimationPID_)){
//                    std::cout << "NULLED THE DRIFT FOR NAN REASONS!! \n";
//                    driftEstimationPID_ = 0.0;
//                }

//                double driftEstimationPIDadder = 0;
//                if (heightDiffPID != 0.0) driftEstimationPIDadder = driftEstimationPID_ * (heightDiffPID/fabs(heightDiffPID));
//                else driftEstimationPIDadder = driftEstimationPID_;

                // TESTING, adjusted this, check consequences..  // HACKED ALSO IN IF ARGUMENT!!! // Hacked from minus to plus!!!!
                if(applyFrameCorrection_ && !isnan(heightDiffPID)) heightDifferenceFromComparison_ = heightDiffPID;// + 0 * driftEstimationPIDadder;// - driftEstimationPIDadder;//  driftEstimationPID_; //! HACKED FOR TESTING< ABSOLUTELY TAKE OUT AGAIN!!!
                else heightDifferenceFromComparison_ = 0.0;
                //if(applyFrameCorrection_ && !footTipOutsideBounds_)

                // For Tuning.
                //performance_assessment_msg_.third_measure = heightDifferenceFromComparison_;

                // Kalman Filter based offset calculation.
                //auto diffTuple = differenceCalculationUsingKalmanFilter();
                //estimatedKalmanDiffIncrement_ =  std::get<0>(diffTuple);
                //estimatedKalmanDiff_ += estimatedKalmanDiffIncrement_;
                //PEstimatedKalmanDiffIncrement_ = std::get<1>(diffTuple);
                //std::cout << "Kalman Filtered Difference Estimate!" << estimatedKalmanDiff_ << "VS. PID Difference: " << heightDifferenceFromComparison_ << std::endl;
                // TESTING:
                //heightDifferenceFromComparison_ = estimatedKalmanDiff_;



                //performance_assessment_msg_.fifth_measure = driftEstimationPID_;

                double second_measure_factor = -10000.0;
                //performance_assessment_msg_.second_measure = driftEstimationPID_ * weightedDifferenceVector_[5] * second_measure_factor;

             //   std::cout << "weightedDifferenceVector!!: LAst element: " << weightedDifferenceVector_[5] << " Drift Est PID!!!:::::: " << (double)driftEstimationPID_ << std::endl;
             //   std::cout << "heightDifferenceFromComparison::: " << heightDifferenceFromComparison_ << std::endl;

                // TODO: Add some low pass filtering: FUTURE: Low passing for Mode changing..
                // Low pass filtering for robust high grass detection ****************************************************
                //if (grassDetectionHistory_.size() > 2) grassDetectionHistory_.erase(grassDetectionHistory_.begin());
                //if (performance_assessment_msg_.second_measure > 0.1){
                //    std::cout << "ADDED A 1 TO HIGH GRASS FILTERING!!" << std::endl;
                //    grassDetectionHistory_.push_back(1);
                //}
                //else grassDetectionHistory_.push_back(0);

                //int sum_of_elems = 0;
                //for (bool n : grassDetectionHistory_)
                //    sum_of_elems += n;
                //if (sum_of_elems > 0)
                   // std::cout << "HIGH GRASS MODE ACTIVATED!!!!!" << "\n" << "\n" << "\n" << "\n" << "\n" << std::endl;
                // End low pass filtering  ************************************************




            }
            else{
                //if(!isnan(driftEstimationPID_)) heightDifferenceFromComparison_ += driftEstimationPID_; // HACKED AWAY FOR TESTING!
                //std::cout << "heightDifferenceFromComparison_ was Incremented by estimated Drift because NAN was found: " << (double)driftEstimationPID_ << std::endl;

                // TODO: consider to set the height difference to zero when walking out of the map!!
                // Delete the weighted Difference Vector if walking outside of the known mapped area to avoid large corrections when reentering..
                if (weightedDifferenceVector_.size() > 0) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
                else heightDifferenceFromComparison_ = 0.0;
            }
        }
        else{

            std::cout << "heightDifferenceFromComparison_ wasnt changed because tip is not inside map area" << std::endl;


            // Delete the weighted Difference Vector if walking outside of the known mapped area to avoid large corrections when reentering..
            if (weightedDifferenceVector_.size() > 0) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
            else heightDifferenceFromComparison_ = 0.0;
        }
    }

    // Publish frame, offset by the height difference parameter.
    //frameCorrection();

    // Publish the elevation map with the new layer, at the frequency of the stances.
    grid_map_msgs::GridMap mapMessage;
    GridMapRosConverter::toMessage(rawMap_, mapMessage);
    mapMessage.info.header.frame_id = "odom_drift_adjusted"; //! HACKED!!

    //GridMapRosConverter::fromMessage(mapMessage, rawMapCorrected)

    elevationMapCorrectedPublisher_.publish(mapMessage);



    return true;
}

bool ElevationMap::initializeFootTipMarkers()
{
    // Color and shape definition of markers for foot tip ground contact visualization.
    footContactMarkerList_.header.frame_id = elevationMapBoundMarkerList_.header.frame_id = "odom";
    footContactMarkerList_.header.stamp = elevationMapBoundMarkerList_.header.stamp = ros::Time();
    footContactMarkerList_.ns = elevationMapBoundMarkerList_.ns = "elevation_mapping";
    footContactMarkerList_.id = elevationMapBoundMarkerList_.id = 0;
    footContactMarkerList_.type = elevationMapBoundMarkerList_.type = visualization_msgs::Marker::SPHERE_LIST;
    footContactMarkerList_.action = elevationMapBoundMarkerList_.action = visualization_msgs::Marker::ADD;
    footContactMarkerList_.pose.orientation.x = elevationMapBoundMarkerList_.pose.orientation.x = 0.0;
    footContactMarkerList_.pose.orientation.y = elevationMapBoundMarkerList_.pose.orientation.y = 0.0;
    footContactMarkerList_.pose.orientation.z = elevationMapBoundMarkerList_.pose.orientation.z = 0.0;
    footContactMarkerList_.pose.orientation.w = elevationMapBoundMarkerList_.pose.orientation.w = 1.0;
    footContactMarkerList_.scale.x = 0.03;
    footContactMarkerList_.scale.y = 0.03;
    footContactMarkerList_.scale.z = 0.03;
    elevationMapBoundMarkerList_.scale.x = 0.02;
    elevationMapBoundMarkerList_.scale.y = 0.02;
    elevationMapBoundMarkerList_.scale.z = 0.02;
    footContactMarkerList_.color.a = elevationMapBoundMarkerList_.color.a = 1.0; // Don't forget to set the alpha!
    footContactMarkerList_.color.r = elevationMapBoundMarkerList_.color.r = 0.0;
    footContactMarkerList_.color.g = elevationMapBoundMarkerList_.color.g = 1.0;
    footContactMarkerList_.color.b = elevationMapBoundMarkerList_.color.b = 0.7;

    // Visualize the plane, that is fit to the foot tip in a least squares fashion.
    footTipPlaneFitVisualization_.header.frame_id = "odom";
    footTipPlaneFitVisualization_.header.stamp = ros::Time();
    footTipPlaneFitVisualization_.ns = "elevation_mapping";
    footTipPlaneFitVisualization_.id = 0;
    footTipPlaneFitVisualization_.type = visualization_msgs::Marker::CUBE_LIST;
    footTipPlaneFitVisualization_.action = visualization_msgs::Marker::ADD;
    footTipPlaneFitVisualization_.pose.orientation.x = 0.0;
    footTipPlaneFitVisualization_.pose.orientation.y = 0.0;
    footTipPlaneFitVisualization_.pose.orientation.z = 0.0;
    footTipPlaneFitVisualization_.pose.orientation.w = 1.0;
    footTipPlaneFitVisualization_.scale.x = 0.07;
    footTipPlaneFitVisualization_.scale.y = 0.07;
    footTipPlaneFitVisualization_.scale.z = 0.07;
    footTipPlaneFitVisualization_.color.a = 1.0;
    footTipPlaneFitVisualization_.color.r = 0.3;
    footTipPlaneFitVisualization_.color.g = 0.05;
    footTipPlaneFitVisualization_.color.b = 0.55;

    return true;
}

std::tuple<double, double> ElevationMap::filteredDriftEstimation(double diffComparisonUpdate, float estDrift, float PEstDrift)
{
    double predDrift, PPredDrift, measDrift, PMeasDrift;
    if(!isnan(diffComparisonUpdate)){

      //  measurement = newheightdiff - oldheightdiff
        double diffMeasurement = diffComparisonUpdate - oldDiffComparisonUpdate_;
        std::cout << "diffComparisonUpdate: " << diffComparisonUpdate << std::endl;
        std::cout << "est Drift: " << estDrift << std::endl;
        std::cout << "diffMeasurement: " << diffMeasurement << std::endl;
        float R = 0.002;
        float Q = 0.2;
    //    // Prediction Step.
        predDrift = estDrift;
        PPredDrift = PEstDrift + Q;

    //    // Measurement Step.
        double K = PPredDrift / (PPredDrift+R);
        std::cout << "K: " << K << std::endl;
        measDrift = predDrift + K * diffMeasurement;
        PMeasDrift = PPredDrift - K * PPredDrift;

        std::cout << "mean measDrift Increment: " << measDrift << " per stance" << std::endl;
        // Attention: if nans are there, then drift is set to zero!
    }
    else{
        measDrift = estDrift;
        PMeasDrift = PEstDrift;
    }
    return std::make_tuple(measDrift, PMeasDrift);
}

std::tuple<double, double, double> ElevationMap::getFusedCellBounds(const Eigen::Vector2d& position, const Eigen::Array2d& length)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipComparison(footTipStanceComparisonMutex_);
   // boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
   // boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    float upperFused, lowerFused, elevationFused;
    bool doFuseEachStep = true;
    if(!isnan(heightDifferenceFromComparison_) && doFuseEachStep){
        fuseArea(position, length);
        elevationFused = fusedMap_.atPosition("elevation", position);
        lowerFused = fusedMap_.atPosition("lower_bound", position);
        upperFused = fusedMap_.atPosition("upper_bound", position);
    }
    return std::make_tuple(lowerFused, elevationFused, upperFused);
}

bool ElevationMap::frameCorrection()
{
    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
    //boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);

    // Transform Broadcaster for the /odom_z_corrected frame.
    tf::Transform odomMapTransform;

    odomMapTransform.setIdentity();

    if (!isnan(heightDifferenceFromComparison_)) odomMapTransform.getOrigin()[2] += heightDifferenceFromComparison_;
    else std::cout << heightDifferenceFromComparison_ << " <- height diff is this kind of NAN for some reason? \n ? \n ? \n";

    //ros::Time stamp = ros::Time().fromNSec(fusedMap_.getTimestamp());

    //std::cout << "TIMESTAMP PUBLISHED THE odom_drift_adjusted TRANSFORM!!: " << stamp << std::endl;

    mapCorrectedOdomTransformBroadcaster_.sendTransform(tf::StampedTransform(odomMapTransform,
                                          ros::Time().fromNSec(rawMap_.getTimestamp()), "odom", "odom_drift_adjusted"));

    return true;
}

float ElevationMap::differenceCalculationUsingPID()
{
    // TODO: Tune these, they are only guessed so far.
    float kp = kp_;
    float ki = ki_;
    float kd = kd_; // Trying PI controller for now.. (Quite good between 0 and -0.1)

    // Nan prevention.
    if(weightedDifferenceVector_.size() < 1 || isnan(weightedDifferenceVector_[1])) return 0.0; // HACKED!!!!!
    else if(weightedDifferenceVector_.size() > 1){
        // Calculate new total diff here
        double totalDiff = 0.0;
        for (auto& n : weightedDifferenceVector_)
            totalDiff += n;
        double meanDiff = totalDiff / float(weightedDifferenceVector_.size());
  //      std::cout << "MeanDiff: " << meanDiff << std::endl;
        return kp * weightedDifferenceVector_[weightedDifferenceVector_.size()-1] + ki * meanDiff +
                kd * (weightedDifferenceVector_[weightedDifferenceVector_.size()-1] -
                weightedDifferenceVector_[weightedDifferenceVector_.size()-2]);
    }
    else return 0.0;
}

double ElevationMap::driftCalculationUsingPID(std::string tip){
    //
    double totalDifference = 0.0;
    for(unsigned int i = PIDWeightedDifferenceVector_.size()-1; i > 0; --i){ // HACKED AGAIN HACKED
        double difference = PIDWeightedDifferenceVector_[i] - PIDWeightedDifferenceVector_[i-1];
        totalDifference += difference;
    }
    double meanDifference = 0.0;
    if (PIDWeightedDifferenceVector_.size() > 2) meanDifference = totalDifference / double(PIDWeightedDifferenceVector_.size() - 2.0);
    else{
        meanDifference = 0.0;
    //    std::cout << "ATTENTIONATTENTION \n" << "******* \n" << "********* \n";
    }
    std::cout << "mean DRIFT PER STANCE USING PID !!! " << meanDifference << std::endl;
    if (PIDWeightedDifferenceVector_.size() < 10){ // HACKED
        meanDifference = 0.0;
        std::cout << "NULLED FOR VECTOR SIZE REASONS!! " << PIDWeightedDifferenceVector_.size() << "\n";
    }

    if (heightDifferenceFromComparison_ != 0.0 && !isnan(heightDifferenceFromComparison_)) return meanDifference *
            (heightDifferenceFromComparison_ / fabs(heightDifferenceFromComparison_)); // Check sign policy!!!
    else return 0.0;
}

float ElevationMap::gaussianWeightedDifferenceIncrement(double lowerBound, double elevation, double upperBound, double diff)
{
    diff -= heightDifferenceFromComparison_;

    // New Stuff for testing!! ********************************************************************************************************
    //! For testing
    if(elevation + diff < lowerBound + 0.9 * (elevation - lowerBound)){
      //  std::cout << "************************************* SOFT LOWER!! **************************!!!!!!!!!!!!!!!!!!!!" << std::endl;
        grassDetectionHistory2_.push_back(1);
    }
    else{
        grassDetectionHistory2_.push_back(0);
    }
    if (grassDetectionHistory2_.size() > 12) grassDetectionHistory2_.erase(grassDetectionHistory2_.begin());

    // TODO: At some point create a triger fct, that does this..
    int triggerSum = 0;
    for (auto n : grassDetectionHistory2_){
        triggerSum += n;
    }

    // DEBugging:
    if (triggerSum > 3){
      //  std::cout << "<<<<<<<<>>>>>>>>> \n" << "<<<<<<<<<<>>>>>>>> \n" << "<<<<<<<<<<>>>>>>>>> \n";
        highGrassMode_ = true; // Hacked!!!!
        applyFrameCorrection_ = false;
    }
    else{
      //  std::cout << "!!!! \n" << "!!!! \n" << "!!!! \n";
        highGrassMode_ = false;
        applyFrameCorrection_ = true;
    }

    double footTipVal;
    // Get normalized foot tip position within bounds.
    if(diff < 0.0){
        footTipVal = diff / fabs(elevation - lowerBound);
    }
    else{
        footTipVal = diff / fabs(elevation - upperBound);
    }

    // Write to file to visualize in Matlab.

    //writeFootTipStatisticsToFile(footTipVal, "/home/timon/FootTipStatistics.txt");

    // TODO: write the foot tip characterizing numbers to a file to histogram plot it in Matlab for discussion..

    // End New *******************************************************************************************************************


    // Error difference weighted as (1 - gaussian), s.t. intervall from elevation map to bound contains two standard deviations

    // Initialize
    double weight = 0.0;

    float standardDeviationFactor = weightingFactor_;  //! THIS MAY BE A LEARNING FUNCTION IN FUTURE!! // CHANGED

    if(diff < 0.0){
        weight = normalDistribution(diff, 0.0, fabs(elevation-lowerBound)*standardDeviationFactor);

        // For Coloration
        if(elevation + diff < lowerBound){

        //    std::cout << "******************************************** LOWER ******************************" << std::endl;
            footTipOutsideBounds_ = true;
        }
        else footTipOutsideBounds_ = false;

    }
    else{
        weight = normalDistribution(diff, 0.0, fabs(elevation-upperBound)*standardDeviationFactor);

        // For Coloration
        if(elevation + diff > upperBound) footTipOutsideBounds_ = true;
        else footTipOutsideBounds_ = false;
    }

    // Constrain to be maximally 1.
    if(weight > 1.0) weight = 1.0; // For security, basically not necessary (as the weighting term is left away in the normalDistribution function)

    usedWeight_ = (1.0 - weight);

    // TOOD: code function that does sigmoid droppoff between 1.5 and 2 times the confidence bound -> no weight in this case -> no correction (Marco Hutters comment..)



   // std::cout << "WEIGHT: " << (1-weight) << " diff: " << diff << " elevation: " << elevation <<
   //              " lower: " << lowerBound << " upper: " << upperBound << std::endl;
    return (1.0 - weight) * diff;
}

std::tuple<double, double> ElevationMap::differenceCalculationUsingKalmanFilter()
{
    double predDiff, PPredDiff, measDiff, PMeasDiff;
    if(weightedDifferenceVector_.size() > 0 && !isnan(weightedDifferenceVector_[0])){

      //  measurement = newheightdiff - oldheightdiff
        double diffMeasurement = weightedDifferenceVector_[0] - estimatedKalmanDiff_;
     //   std::cout << "WeightedDifferenceVector inside Kalman Filter: " << weightedDifferenceVector_[0] << std::endl;
        float R = 0.95;
        float Q = 0.05;
    //    // Prediction Step.
        predDiff = estimatedKalmanDiffIncrement_;
        PPredDiff = PEstimatedKalmanDiffIncrement_ + Q;

    //    // Measurement Step.
        double K = PPredDiff / (PPredDiff+R);
        measDiff = predDiff + K * diffMeasurement;
        PMeasDiff = PPredDiff - K * PPredDiff;

     //   std::cout << "mean measKalmanDiff: " << measDiff << std::endl;
    }
    else{
        measDiff = estimatedKalmanDiffIncrement_;
        PMeasDiff = PEstimatedKalmanDiffIncrement_;
    }
    return std::make_tuple(measDiff, PMeasDiff);

}

float ElevationMap::normalDistribution(float arg, float mean, float stdDev)
{
    double e = exp(-pow((arg-mean),2)/(2.0*pow(stdDev,2)));
    //Leaving away the weighting, as the maximum value should be one, which is sensible for weighting.
    return e; // (stdDev * sqrt(2*M_PI));
}

bool ElevationMap::updateFootTipBasedElevationMapLayer(int numberOfConsideredFootTips)
{
    // Get parameters used for the plane fit through foot tips.
    Eigen::Vector2f planeCoeffs = getFootTipPlaneFitCoeffcients();
   // std::cout << "These are the coeffs form the GETTER function: a->" << planeCoeffs(0) << " b->" << planeCoeffs(1) << std::endl;
    Eigen::Vector3f meanOfAllFootTips = getMeanOfAllFootTips();


    double summedErrorValue = 0;
    for (GridMapIterator iterator(rawMap_); !iterator.isPastEnd(); ++iterator) {

        Position posMap;
        rawMap_.getPosition(*iterator, posMap);

        bool complicatedVersionOfFootTipLayer = false;
        if(complicatedVersionOfFootTipLayer){
            std::vector<geometry_msgs::Point>::iterator markerListIterator = footContactMarkerList_.points.end();
            if (footContactMarkerList_.points.size() <= 10) numberOfConsideredFootTips = footContactMarkerList_.points.size();
            double totalWeight = 0;
            double factor = 20.0;
            std::vector<double> weightVector, distanceVector;
            for (unsigned int j = 0; j < numberOfConsideredFootTips; ++j){

                --markerListIterator;
                //std::cout << markerListIterator->z << std::endl;
                double distance = sqrt(pow(fabs(markerListIterator->x - posMap[0]),2) + pow(fabs(markerListIterator->y - posMap[1]),2));
                //std::cout << distance << std::endl;

                distanceVector.push_back(distance);
                totalWeight += 1.0/exp(factor*distance); // Changed to exp
                weightVector.push_back(1.0/exp(factor*distance)); // Changed to exp
            }
            double cellHeight = 0;
            markerListIterator = footContactMarkerList_.points.end();
            markerListIterator--;
            for (unsigned int i = 0; i < weightVector.size(); ++i){
                 cellHeight += markerListIterator->z * (weightVector[i] / totalWeight); // TODO: stronger functional dependency, e.g. exp..
                 markerListIterator--;
            }

            //if (rawMap_.isInside(posMap))) rawMap_.atPosition("foot_tip_elevation", posMap) = cellHeight;

            // TODO: calculate differences to elevation map corrected and colorize it accordingly..


            // HERE: get the height of the plane fit map layer and add it ..
        }
        // Set the height of the plane fit for each cell.
        double cellHeightPlaneFit = 0.0;
        if (!isnan(planeCoeffs(0)) && !isnan(planeCoeffs(1)) && !isnan(meanOfAllFootTips(0)) && !isnan(meanOfAllFootTips(1))) {
            cellHeightPlaneFit = -(posMap(0) - meanOfAllFootTips(0)) * planeCoeffs(0) - (posMap(1) - meanOfAllFootTips(1)) * planeCoeffs(1) + meanOfAllFootTips(2); // Hacked the sign!!!!
        }

        // Calculate Difference measure in restricted area around foot tips (the last two)
        double threshold = 0.4;
        if (rawMap_.isInside(posMap)){// && !isnan(rawMap_.atPosition("elevation", posMap))){
           // if (distanceVector[0] < threshold || distanceVector[1] < threshold)
                //if (rawMap_.isInside(posMap)) rawMap_.atPosition("foot_tip_elevation", posMap) = cellHeight; // Hacked to only update restricted area around foot tips..
                //if (rawMap_.isInside(posMap)) rawMap_.atPosition("foot_tip_elevation", posMap) = (0.7 * cellHeightPlaneFit + 0.3 * cellHeight); // Hacked to get the plane fit version only! //! Hacked, such that half plane fit half the complicated one..
                if (rawMap_.isInside(posMap) && !isnan(cellHeightPlaneFit)) rawMap_.atPosition("foot_tip_elevation", posMap) = 1.0 * cellHeightPlaneFit + 0.0 * rawMap_.atPosition("foot_tip_elevation", posMap);
                // Low Pass filtering of plane fit only prediction..
               // summedErrorValue += fabs(cellHeight - (rawMap_.atPosition("elevation", posMap) + heightDifferenceFromComparison_)); // Hacked a little error to check sensitivity
            // Consider adding the offset, as the non moved one may be considered..
            //std::cout << "Difference in height, what about offset?: " << fabs(cellHeight - rawMap_.atPosition("elevation", posMap)) << std::endl;
        }
    }

    // TODO: getthe foot tip layer to be at all spots, not only where the raw map is definded..
    // TODO: consider low pass filtering, i.e. 0.7, 0.3 to avoid left-right motion..


    std::cout << "Total ERROR VALUE OF FOOT TIP ELEVATION MAP VS CAMERA ELEVATION MAP: " << summedErrorValue << std::endl;
    return true;
}

bool ElevationMap::performanceAssessmentMeanElevationMap()
{
    // Choose area to cut out (to be left aside for performance assessment, i.e. nonplanar obstacles)
    double xMinCutoff, xMaxCutoff, yMinCutoff, yMaxCutoff;
    xMinCutoff = yMinCutoff = -100000000000;
    xMaxCutoff = yMaxCutoff = 100000000000;

    // Set them.
    xMinCutoff = 0.4;
    xMaxCutoff = 1.83;


    // Calculate the summed square deviation fom the mean elevation map height (note: elevation map in frame odom is considered, not in odom_drift_adjusted)
    double totalElevationMapHeight = 0;
    double performanceAssessment = 0;
    int counter = 0;
    for (GridMapIterator iterator(rawMap_); !iterator.isPastEnd(); ++iterator) {

        Position3 posMap3;

        rawMap_.getPosition3("elevation", *iterator, posMap3);

        // TEST:
        //if(posMap3[0] > 0.000000001) std::cout << posMap3[0] << std::endl;
        // END TEST

        if (posMap3[2] > 0.000000001 && posMap3[0] > 0.000000001){
            if (!(posMap3[0] < xMaxCutoff && posMap3[0] > xMinCutoff || posMap3[0] > 2.6)){
                totalElevationMapHeight += posMap3[2];
                counter++;  // TODO: get no of indeces directly..

                //if (posMap3[2] < 0.00001) std::cout << "SMALL SMALL ELEVATION MAP PROBABLY ZERO! " << posMap3[2] << std::endl;
            }
        }
    }
   // std::cout << "MEAN:                                " << totalElevationMapHeight / double(counter) << std::endl;
    double mean = totalElevationMapHeight / double(counter);
    for (GridMapIterator iterator(rawMap_); !iterator.isPastEnd(); ++iterator) {
        Position3 posMap3;
        rawMap_.getPosition3("elevation", *iterator, posMap3);
        if (posMap3[2] > 0.000000001 && posMap3[0] > 0.000000001){
            if (!(posMap3[0] < xMaxCutoff && posMap3[0] > xMinCutoff || posMap3[0] > 2.6)){

                performanceAssessment += pow(posMap3[2]-mean, 2);
            }
        }
    }
    // TODO: Cut out are (of ostacle..)

   // std::cout << "PERFORMANCE ASSESSMENT MEAN:                                " << performanceAssessment/counter << std::endl;

    // Add a factor for scaling in rqt plot
    double factor = 200.0;

    performanceAssessmentFlat_ += factor * performanceAssessment/counter;
    //performance_assessment_msg_.fourth_measure = factor * performanceAssessment/counter;

    return true;
}

bool ElevationMap::writeFootTipStatisticsToFile(double& footTipVal, std::string filename) {
    std::cout << "WRITING TO FILE" << std::endl;
    // Open the writing file.
    ofstream myfile;
    myfile.open (filename, ios::app);
    myfile << "; " << footTipVal;
    myfile.close();
    return true;
}

bool ElevationMap::proprioceptiveRoughnessEstimation(std::string tip){


    //! TODO next! Roughness estimate:
    //! - variance of differences of consecutive placements of the same foot tip
    //! - plane fit into the four feet, deviation is a roughness measure as well
    //! - other correlation between differences? Think of that..



    // Only consider area where robot is trotting for now.
    //if (meanStance_(0) < 1.3 || meanStance_(0) > 2.75){
    if (tip == "left"){
        leftStanceVector_.push_back(meanStance_);
        if (leftStanceVector_.size() > 2) leftStanceVector_.erase(leftStanceVector_.begin());
    }
    if (tip == "right"){
        rightStanceVector_.push_back(meanStance_);
        if (rightStanceVector_.size() > 2) rightStanceVector_.erase(rightStanceVector_.begin());
    }
    if (tip == "lefthind"){
        leftHindStanceVector_.push_back(meanStance_);
        if (leftHindStanceVector_.size() > 2) leftHindStanceVector_.erase(leftHindStanceVector_.begin());
    }
    if (tip == "righthind"){
        rightHindStanceVector_.push_back(meanStance_);
        if (rightHindStanceVector_.size() > 2) rightHindStanceVector_.erase(rightHindStanceVector_.begin());
    }
    //}



    bool writeHorizontalFootTipEstimationStatisticsToFile = false;
    if(writeHorizontalFootTipEstimationStatisticsToFile){
        if (leftStanceVector_.size() > 1 && tip == "left"){
            double diff = double(leftStanceVector_[1](0)-leftStanceVector_[0](0));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
        else if(rightStanceVector_.size() > 1 && tip == "right"){
            double diff = double(rightStanceVector_[1](0)-rightStanceVector_[0](0));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
        else if(leftHindStanceVector_.size() > 1 && tip == "lefthind"){
            double diff = double(rightStanceVector_[1](0)-rightStanceVector_[0](0));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
        else if(rightHindStanceVector_.size() > 1 && tip == "righthind"){
            double diff = double(rightStanceVector_[1](0)-rightStanceVector_[0](0));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
    }
    proprioceptiveVariance(tip); //! Mean drift value could be found by calibration during flat ground walking

}

bool ElevationMap::proprioceptiveVariance(std::string tip){

    double diff = 0.0;
    double meanDrift = 0.467;
    if (leftStanceVector_.size() > 1 && tip == "left"){
        diff = double((leftStanceVector_[1](0) - leftStanceVector_[0](0)));
    }
    else if(rightStanceVector_.size() > 1 && tip == "right"){
        diff = double((rightStanceVector_[1](0) - rightStanceVector_[0](0)));
    }
    else if(leftHindStanceVector_.size() > 1 && tip == "lefthind"){
        diff = double((leftHindStanceVector_[1](0) - leftHindStanceVector_[0](0)));
    }
    else if(rightHindStanceVector_.size() > 1){
        diff = double((rightHindStanceVector_[1](0) - rightHindStanceVector_[0](0)));
    }

    feetUnseenVarianceVector_.push_back(diff);
    if (feetUnseenVarianceVector_.size() > 12) feetUnseenVarianceVector_.erase(feetUnseenVarianceVector_.begin());

    // Calculate Variance.
    double total = 0.0;
    double totalSquared = 0.0;
    for (auto& n : feetUnseenVarianceVector_){
        total += n;
        totalSquared += pow(n, 2);
    }
    double varianceConsecutiveFootTipPositions = totalSquared / (double)feetUnseenVarianceVector_.size() - pow(total/(double)feetUnseenVarianceVector_.size(), 2);
    double mean = total/(double)feetUnseenVarianceVector_.size();

    // TODO: add 4 feet plane fit variance calculation..
    // TODO: think about what slope is adding to roughness estimate..
    // TODO: and other roughness assessment measures

  //  std::cout << "varianceConsecutiveFootTipPositions::::::::::::::::::::::: " << varianceConsecutiveFootTipPositions << std::endl;
  //  std::cout << "Mean::::::::::::::::::::::::::: " << mean << std::endl;
    // TODO: Consider low pass filtering effect on roughness estimation!

    // Plane fitting variance:



    //    for all 4 feet..
    // TODO: Plane Fitting for the 4 foot tips to get deviation from it (formulate as variance if squared difference, is that valid?)
    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0 &&
            leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0){ // Check, that at least one stance position per foot tip is available.
        Eigen::Vector3f leftTip, rightTip, leftHindTip, rightHindTip;
        leftTip = leftStanceVector_[leftStanceVector_.size()-1];
        rightTip = rightStanceVector_[rightStanceVector_.size()-1];
        leftHindTip = leftHindStanceVector_[leftHindStanceVector_.size()-1];
        rightHindTip = rightHindStanceVector_[rightHindStanceVector_.size()-1];

        double meanZelevation = (leftTip(2) + rightTip(2) + leftHindTip(2) + rightHindTip(2)) / 4.0;
    //    std::cout << "meanzvaluefrom foottips: " << meanZelevation << std::endl;

        Eigen::Matrix2f leftMat;
        leftMat(0, 0) = pow(leftTip(0),2)+pow(rightTip(0),2) + pow(leftHindTip(0),2)+pow(rightHindTip(0),2);
        leftMat(1, 0) = leftMat(0, 1) = leftTip(0) * leftTip(1) + rightTip(0) * rightTip(1) +
                leftHindTip(0) * leftHindTip(1) + rightHindTip(0) * rightHindTip(1);
        leftMat(1, 1) = pow(leftTip(1),2)+pow(rightTip(1),2) + pow(leftHindTip(1),2)+pow(rightHindTip(1),2);
        Eigen::Vector2f rightVec;
        rightVec(0) = leftTip(0) * (leftTip(2)-meanZelevation) + rightTip(0) * (rightTip(2)-meanZelevation) +
                leftHindTip(0) * (leftHindTip(2)-meanZelevation) + rightHindTip(0) * (rightHindTip(2)-meanZelevation);
        rightVec(1) = leftTip(1) * (leftTip(2)-meanZelevation) + rightTip(1) * (rightTip(2)-meanZelevation) +
                leftHindTip(1) * (leftHindTip(2)-meanZelevation) + rightHindTip(1) * (rightHindTip(2)-meanZelevation);

        Eigen::Vector2f sol = leftMat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-rightVec);

        setFootTipPlaneFitCoefficients(sol);
        Eigen::Vector3f meanOfAllFootTips;
        for (unsigned int j = 0; j <= 2; ++j) meanOfAllFootTips(j) = 0.25 * (leftTip(j) + rightTip(j) + leftHindTip(j) + rightHindTip(j));
        setMeanOfAllFootTips(meanOfAllFootTips);


        bool visualizeFootTipPlaneFit = true;
        if (visualizeFootTipPlaneFit){

            geometry_msgs::Point p1, p2, p3, p4;
            p1.x = leftTip(0);
            p2.x = rightTip(0);
            p3.x = leftHindTip(0);
            p4.x = rightHindTip(0);
            p1.y = leftTip(1);
            p2.y = rightTip(1);
            p3.y = leftHindTip(1);
            p4.y = rightHindTip(1);

            // Attention: TODO mean position is important!!! (mean x and y..)
            p1.z = -(sol(0) * (leftTip(0) - meanOfAllFootTips(0)) + sol(1) * (leftTip(1) - meanOfAllFootTips(1))) + meanZelevation;
            p2.z = -(sol(0) * (rightTip(0) - meanOfAllFootTips(0)) + sol(1) * (rightTip(1) - meanOfAllFootTips(1))) + meanZelevation;
            p3.z = -(sol(0) * (leftHindTip(0) - meanOfAllFootTips(0)) + sol(1) * (leftHindTip(1) - meanOfAllFootTips(1))) + meanZelevation;
            p4.z = -(sol(0) * (rightHindTip(0) - meanOfAllFootTips(0)) + sol(1) * (rightHindTip(1) - meanOfAllFootTips(1))) + meanZelevation;
            footTipPlaneFitVisualization_.points.push_back(p1);
            footTipPlaneFitVisualization_.points.push_back(p2);
            footTipPlaneFitVisualization_.points.push_back(p3);
            footTipPlaneFitVisualization_.points.push_back(p4);

            // DUDUDUDUDU
            planeFitVisualizationPublisher_.publish(footTipPlaneFitVisualization_);

            // Calculate squared difference from plane:
            double variancePlaneFit = pow(p1.z - leftTip(2), 2) + pow(p2.z - rightTip(2), 2) +
                    pow(p3.z - leftHindTip(2), 2) + pow(p4.z - rightHindTip(2), 2);
      //      std::cout << "variancePlaneFit: " << variancePlaneFit << std::endl;

            geometry_msgs::Twist varianceMsg;
            varianceMsg.linear.x = varianceConsecutiveFootTipPositions;
            varianceMsg.linear.y = variancePlaneFit;
            varianceMsg.linear.z = varianceConsecutiveFootTipPositions + variancePlaneFit;

            varianceMsg.angular.x = getPenetrationDepthVariance();
           // footTipPlaneFitVisualization_.action = visualization_msgs::Marker::;

            // TODO: Do visualize fitted Plane!!
            varianceTwistPublisher_.publish(varianceMsg);
        }

    }
    // TODO: add Layer, that takes variace uncertainty into account.
}

//bool ElevationMap::penetrationDepthEstimation(std::string tip){
//    // TODO: penetration depth estimation here
//}

void ElevationMap::setFootTipPlaneFitCoefficients(Eigen::Vector2f& coeffs){
    footTipPlaneFitCoefficients_ = coeffs;
}

Eigen::Vector2f ElevationMap::getFootTipPlaneFitCoeffcients(){
    return footTipPlaneFitCoefficients_;
}

void ElevationMap::setMeanOfAllFootTips(Eigen::Vector3f& mean){
    meanOfAllFootTips_ = mean;
}

Eigen::Vector3f ElevationMap::getMeanOfAllFootTips(){
    return meanOfAllFootTips_;
}




bool ElevationMap::penetrationDepthVarianceEstimation(std::string tip, double verticalDifference){
    verticalDifferenceVector_.push_back(verticalDifference);
    if (verticalDifferenceVector_.size() > 6) verticalDifferenceVector_.erase(verticalDifferenceVector_.begin());
    double totalVerticalDifference = 0.0;
    double squaredTotalVerticalDifference = 0.0;
    int count = verticalDifferenceVector_.size();
    for (auto& n : verticalDifferenceVector_){
        totalVerticalDifference += n;
        squaredTotalVerticalDifference += pow(n,2);
    }

    double penetrationDepthVariance = squaredTotalVerticalDifference / double(count) +
            pow(totalVerticalDifference / double(count), 2);

    setPenetrationDepthVariance(penetrationDepthVariance);
    return true;
}




bool ElevationMap::writeDataFileForParameterLearning(){
    // Open the writing file.
    ofstream tunefile;
    tunefile.open ("/home/timon/usedParams.txt", ios::app);
    tunefile << kp_ << " " << ki_ << " " << kd_ << " " << weightingFactor_ << "\n";
    // TODO add some calculation of total performance assessment value.
    // Write the tuning params.

    std::cout << "Written Params to file!!!!" << std::endl;

    ofstream perffile;
    perffile.open ("/home/timon/perfAssessment.txt", ios::app);

    // TODO: This value is calculated as:
    perffile << performanceAssessment_ + performanceAssessmentFlat_ << "\n";
    perffile.close();

    return true;
}

// High Grass Functions from here:

bool ElevationMap::updateSupportSurfaceEstimation(){

    // Here all the functions are called and weighting is set..

    // For testing: initialize the support surface layer as the foot tip elevation layer..
    //rawMap_.add("support_surface", rawMap_.get("foot"));

    // TODO: Here get position of foot tips and get indeces and insert them as the starting indices.. Insert them as arguments into the functions..

  //  std::cout << "Called the update function" << std::endl;

    //penetrationDepthContinuityPropagation();
    //terrainContinuityPropagation();


    penetrationDepthContinuityProcessing();
    terrainContinuityProcessing();


    footTipBasedElevationMapIncorporation();




    // TODO: at some point initialize the first two elevation map layers, maybe heavy weight on foot tip only layer..
}

bool ElevationMap::penetrationDepthContinuityPropagation(){

    double penContinuity = 0.98;

    double factorProp = 1.0 - penContinuity; // Multiplier for values of comparison cells
    double factorComp = penContinuity; // Multiplier for values of propagation product cell

    // TESTING:
    grid_map::Index startingIndex(196, 1);
    // ENS TESTS
    cellPropagation(factorProp, factorComp, startingIndex, "penDepth");

    return true;
}

bool ElevationMap::terrainContinuityPropagation(){
    double terrContinuity = 0.75;

    double factorProp = 1.0 - terrContinuity;
    double factorComp = terrContinuity;

    // TESTING:
    grid_map::Index startingIndex(196, 1);
    cellPropagation(factorProp, factorComp, startingIndex, "terr");
    return true;
}

Eigen::Vector3f ElevationMap::getMeanStance(){
    return meanStance_;
}

Eigen::Vector3f ElevationMap::getLatestLeftStance(){
    return leftStanceVector_[leftStanceVector_.size()-1];
}

Eigen::Vector3f ElevationMap::getLatestRightStance(){
    return rightStanceVector_[rightStanceVector_.size()-1];
}

bool ElevationMap::cellPropagation(double factorProp, double factorComp, grid_map::Index& startingIndex, std::string propagationMethod){

    grid_map::Matrix& dataSupp = rawMap_["support_surface"];
    grid_map::Matrix& dataElev = rawMap_["elevation"];
    grid_map::Matrix& dataFoot = rawMap_["foot_tip_elevation"];

    if (leftStanceVector_.size() > 1 && rightStanceVector_.size() > 1){
        if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0){
            // TODO: set sensible starting index..  (Direction Sensitivity!!!) (after showing the principle..)
            Eigen::Vector3f latestLeftStance = getLatestLeftStance();
            Eigen::Vector3f latestRightStance = getLatestRightStance();

            Position posLatestLeft(latestLeftStance(0), latestLeftStance(1));
            Position posLatestRight(latestRightStance(0), latestRightStance(1));

            Index indLeftTip, indRightTip;


            if (supportSurfaceInitializationTrigger_ == false){
                dataSupp = dataElev; // New!!
                supportSurfaceInitializationTrigger_ = true;

                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;
                std::cout << "INITIALIZED, only once" << std::endl;


            }

            rawMap_.getIndex(posLatestLeft, indLeftTip);
            rawMap_.getIndex(posLatestRight, indRightTip);

         //   std::cout << "Indices: Left(0): " << indLeftTip(0) << " Left(1): " << indLeftTip(1) << " right(0) " << indRightTip(0) << "right(1)" << indRightTip(1) << std::endl;

            // Comparison in x direction..
            if (indLeftTip(0) <= indRightTip(0)) startingIndex(0) = indLeftTip(0);
            else startingIndex(0) = indRightTip(0);
        }

        // Here: heavy weighting of the first two rows against foot tip only elevation map! Diagonal in case of diagonal walking direction..
        for (unsigned int i = startingIndex(0); i >= startingIndex(0) - 2; --i){ // Hacked in here..
            for (unsigned int j = startingIndex(1); j < 199; ++j){
                grid_map::Index indexInit(i,j);
                dataSupp(indexInit(0), indexInit(1)) = 0.09 * dataFoot(indexInit(0), indexInit(1)) + 0.91 * dataSupp(indexInit(0), indexInit(1)); // Hacked here..
            }
        }

        double weight;

        // Introduce a version with ending index, as indices are restarting after some distance
        Index endingIndex;
        if (startingIndex(0) - 199 < 1) endingIndex(0) = 200 + (startingIndex(0) - 199); // HACKED FROM 100 to 10
        else endingIndex(0) = startingIndex(0) - 199;
        int i = startingIndex(0) - 3;

        //std::cout << "HERE I ALSO GOT TO!" << std::endl;

        while (i != endingIndex(0)){
        //for (unsigned int i = startingIndex(0); i >= 1 ; --i){   // This was replaced by a while loop
        //    std::cout << "HERE I GOT TO INSIDE LOOP!: " << i <<  std::endl << std::endl;

            for (unsigned int j = startingIndex(1); j < 199; ++j){  // ATTENTION HACKED AROUND HERE..


                //if (isnan(dataElev(i,j))){
                    //std::cout << "zeroed it " << dataElev(i,j) << std::endl;
                //    dataElev(i,j) = 3.0;
                //}

                int totalWeight = 0; // The total weight of the comparison cells in int (without the factor)
                double totalValue = 0.0;
                double totalAbsValue = 0.0; // Testing, for more stable terrain continuity propagation
                for (unsigned int n = 1; n <= 2; ++n){
                    for (int m = -1; m <= 1; ++m){

                        grid_map::Index indexComp(i+n,j+m);
                        weight = 4 - n - fabs(m);

                        // validity checks here..
                       // std::cout << "Here I came to.." << std::endl;
                        // Realized!!: Iteration is flawed, indices start to repeat at some point -> new stopping criterion!!!

                        grid_map::Index indexHelp(indexComp(0)+1, indexComp(1));

                        totalWeight += weight;

                        if (rawMap_.isValid(indexComp)){ // How to characterize a nan to be in the area of interest and to fill the hole?

                            // HAcked Foot in here instead of supp..
                            if (propagationMethod == "penDepth") totalValue += weight * (dataElev(indexComp(0), indexComp(1)) - dataSupp(indexComp(0), indexComp(1))); // CHenged back from foot to supp
                            if (propagationMethod == "terr" && rawMap_.isValid(indexHelp)) totalValue += weight * (dataSupp(indexComp(0), indexComp(1)) - dataSupp(indexComp(0)+1, indexComp(1)));
                            if (propagationMethod == "terr" && rawMap_.isValid(indexHelp)) totalAbsValue += weight * (dataSupp(indexComp(0), indexComp(1))); // HACKED
                        }
                    }
                }

                grid_map::Index indexProp(i,j);

                //std::cout << "i: " << i << "j: " << j << std::endl;

                double propagatedDifference;
                if(propagationMethod == "penDepth" && totalWeight > 0){
                    propagatedDifference = factorProp * (dataElev(indexProp(0), indexProp(1)) - dataSupp(indexProp(0), indexProp(1))) +
                            factorComp * (totalValue / (double)totalWeight);
                    dataSupp(indexProp(0), indexProp(1)) = dataElev(indexProp(0), indexProp(1)) - propagatedDifference;

                    // For robustness. (maybe unnecessary..)
                    dataSupp(indexProp(0), indexProp(1)) = fmin(dataSupp(indexProp(0), indexProp(1)), dataElev(indexProp(0), indexProp(1)));
                }


                grid_map::Index indexHelp2(indexProp(0)+1, indexProp(1)); // For nan suppression, prelim.

                if(propagationMethod == "terr" && rawMap_.isValid(indexHelp2) && totalWeight > 0){

                    propagatedDifference = factorProp * (dataSupp(indexProp(0), indexProp(1)) - dataSupp(indexProp(0) + 1, indexProp(1))) +
                            factorComp * (totalValue / (double)totalWeight);


                    double absVersion = factorProp * dataSupp(indexProp(0), indexProp(1)) + factorComp * (totalAbsValue / (double)totalWeight); // HACKED

                    dataSupp(indexProp(0), indexProp(1)) = absVersion + 0.9 * propagatedDifference; // New version Hacked factor 0.6 into this!

                    // Weighted version for testing. HACKED not nice, reason about this..
                    //dataSupp(indexProp(0), indexProp(1)) = dataSupp(indexProp(0) + 1, indexProp(1)) + propagatedDifference; // This is the original!



                    //dataSupp(indexProp(0), indexProp(1)) = factorProp * dataSupp(indexProp(0), indexProp(1)) + factorComp * (totalAbsValue / (double)totalWeight); // HACKED




                    // For robustness. (maybe unnecessary..)
                    dataSupp(indexProp(0), indexProp(1)) = fmin(dataSupp(indexProp(0), indexProp(1)), dataElev(indexProp(0), indexProp(1)));

                    //if (isnan(propagatedDifference)) std::cout << "NAN in diff calculation was the reason!!!! totalweight: " << totalWeight << std::endl;

                    // Hacking for debugging:
                    // Check which ones are nans, and how to differenciate them from the outliers..
                    //if (isnan(dataElev(indexProp(0), indexProp(1)))) dataSupp(indexProp(0), indexProp(1)) = 0.0;

                }
                // TODO: add some security checks for nans and stuff..
            }

        --i;
        //std::cout << "After decrementing I: " << i << std::endl;
        if (i < 0) i = 200 + i;
        //std::cout << "Continuing: " << i << std::endl;
        }
    }


    // Incorporate foot tip elevation. -> @ some point, tune this according to:
    // discontinuity estimations..
    //dataSupp = 0.98 * dataSupp + 0.02 * dataFoot;


    return true;
    // Necessary arguments, i.e. diffs: factor1, factor2, starting x starting y, terrain/pendepth.. do the faster version here..
}

bool ElevationMap::footTipBasedElevationMapIncorporation(){

}

// For using it in the penetration depth estimation.
void ElevationMap::setPenetrationDepthVariance(double penetrationDepthVariance){
    if (!isnan(penetrationDepthVariance)) penetrationDepthVariance_ = penetrationDepthVariance;
}

double ElevationMap::getPenetrationDepthVariance(){
    return penetrationDepthVariance_;
}

bool ElevationMap::penetrationDepthContinuityProcessing(){

    // Mutex here for safe multithreading
    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

    //grid_map::Matrix& dataSupp = rawMap_["support_surface"];
    //grid_map::Matrix& dataElev = rawMap_["elevation"];
    //grid_map::Matrix& dataFoot = rawMap_["foot_tip_elevation"];
    //grid_map::Matrix& dataVegHeight = rawMap_["vegetation_height"];
    //grid_map::Matrix& dataSupSurf = rawMap_["support_surface"];
    //grid_map::Matrix& dataSupSurfSmooth = rawMap_["support_surface_smooth"];


    // initialization of dataSupp.
    //if (supportSurfaceInitializationTrigger_ == false){
    //    dataSupp = dataElev;
    //    supportSurfaceInitializationTrigger_ = true;
    //}





    //grid_map::Matrix dataElevInpainted; // = rawMap_["elevation"]; // HACKED

    //InpaintFilter<grid_map::GridMap> input;

    //grid_map::GridMapCvProcessing proc;

   // grid_map::BufferNormalizerFilter<grid_map::Matrix> buffNorm;

    //grid_map::GridMapCvConverter conv;

    //GridMap inputMap = dataElev;


   // std::cout << "data Elev Size ---------: " << dataElev.rows() << "         " <<
   //              dataElev.cols() << std::endl;



   // std::cout << "Some value: " << dataElev(22,22) << std::endl;

    // Testing for inpainting..
//    for (unsigned int i = 0; i < 199; ++i){
//        for (unsigned int j = 0; j < 199; ++j){
//            Index temp(i,j);
//            if (isnan(rawMap_.at("elevation", temp))) rawMap_.at("elevation", temp) = 1.0;
//            else std::cout << "was not nan!!!" << std::endl;
//        }
//    }
    // End Testing..

    //GridMap outMap1;

   // Length len(4.0, 4.0);
   // Position pos(3.0, 0.0); // Hacking to test influence..

   // inpaintedMap.setGeometry(len, 0.02, pos);




   // rawMap_.g
    // Logics: vegetation height -> vegetation height smoothed -> support surf -> support surf smoothed..






    //inpaintedMap.setStartIndex(rawMap_.getStartIndex()); // Tests..

   // Position rawMapStartIndexPosition;
   // rawMap_.getPosition(rawMap_.getStartIndex(), rawMapStartIndexPosition);

   // inpaintedMap.move(rawMapStartIndexPosition);

   // rawMap_.getP

//    inpaintedMap.setGeometry(); // Todo..



    //inpaintedMap = rawMap_;  // Hacked, trying to solve the index issue..

   // inpaintedMap.setGeometry(rawMap_.g);

    //inpaintedMap = rawMap_; // Testing ..

    //inpaintedMap.setStartIndex(rawMap_.getStartIndex());

   // inpaintedMap.move(rawMap_.getPosition());


    //dataVegHeight = dataElev - dataSupSurfSmooth;


    //std::cout << "Jow// \n";
    //if(!filterChain_.update(rawMap_, inpaintedMap)) return false;  // Check this stuff..

    //rawMap_["vegetation_height"] = dataVegHeight;

    //filterChain_.update(rawMap_, outMap1);
    // Check the layers of the first map..

   // std::cout << "Layer 0: " << inpaintedMap.getLayers()[0] << " Layer 1: " << inpaintedMap.getLayers()[1] << std::endl;
   // std::cout << "No. of layers: " << inpaintedMap.getLayers().size() << std::endl;
   // std::cout << "Last Layer: " << inpaintedMap.getLayers()[inpaintedMap.getLayers().size() - 1] << std::endl;

    //rawMap_["vegetation_height_smooth"] = outMap1["vegetation_height_smooth"];


    //grid_map::Matrix& dataVegHeightSmooth = rawMap_["vegetation_height_smooth"];
    //dataSupSurf = dataElev - dataVegHeightSmooth;
    //rawMap_["support_surface"] = rawMap_["elevation"] - outMap1["vegetation_height_smooth"];



    //rawMap_["support_surface"] = dataSupSurf;
  //  inpaintedMap.get("vegetation_height_smooth"]);

    GridMap outMap2;

    //std::cout << "Here I got to!!!! 0000" << std::endl;

    filterChain2_.update(rawMap_, outMap2);  // Check this stuff..

   // std::cout << "Here I got to!!!! " << std::endl;



    // Display the smoothed vegetation height.
    //outMap2["vegetation_height_smooth"] = dataVegHeightSmooth;

   // std::cout << "Here I got to!!!! 1212" << std::endl;





    // Simple foot tip embedding (soon transfer to function..)
    double radius = 0.1; // Maximum search radius for spiralling search in order to find the closest map element in case if nan is present..
    Position3 footTipLeft = getFrontLeftFootTipPosition();
    Position3 footTipRight = getFrontRightFootTipPosition();
    Position leftTipHorizontal(footTipLeft(0), footTipLeft(1));
    Position rightTipHorizontal(footTipRight(0), footTipRight(1));
    double verticalDiffLeft, verticalDiffRight;
    if(outMap2.isInside(leftTipHorizontal) && outMap2.isInside(rightTipHorizontal)){
        if (!isnan(outMap2.atPosition("support_surface_smooth", leftTipHorizontal))){
            verticalDiffLeft = footTipLeft(2) - outMap2.atPosition("support_surface_smooth", leftTipHorizontal); // Hacked to rawMap_
            std::cout << "Not spiralled!!, vertical diff left: " << verticalDiffLeft << std::endl;
        }
        else verticalDiffLeft = getClosestMapValueUsingSpiralIterator(outMap2, leftTipHorizontal, radius, footTipLeft(2)); // New experiment.. Wrong, not difference yet!!!
        //else verticalDiffLeft = 0.0;
        if (!isnan(outMap2.atPosition("support_surface_smooth", rightTipHorizontal))){
            verticalDiffRight = footTipRight(2) - outMap2.atPosition("support_surface_smooth", rightTipHorizontal); // Hacked to rawMap_
            std::cout << "Not spiralled!!, vertical Diff right: " << verticalDiffRight << std::endl;
        }
        //else verticalDiffRight = 0.0; // TODO: check what to do in such a case.. (search a small circular radius)
        // Idea: check circular grid map iterator for first entry.. Spiral!!!!
        else verticalDiffRight = getClosestMapValueUsingSpiralIterator(outMap2, rightTipHorizontal, radius, footTipRight(2)); // New Experiment..
        //else verticalDiffRight = 0.0;
    }
    else verticalDiffRight = verticalDiffLeft = 0.0;

    double meanDiffEmbedding = (verticalDiffLeft + verticalDiffRight) / 2.0;

    std::cout << "Mean Diff Embedding: " << meanDiffEmbedding << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Mean Diff Embedding: " << meanDiffEmbedding << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

    if (verticalDiffRight != 0.0 && verticalDiffLeft != 0.0){
        outMap2.add("additional_layer");
        outMap2["additional_layer"].setConstant(meanDiffEmbedding);
        grid_map::Matrix& dataOutputSupSurfSmooth = outMap2["support_surface_smooth"];
        grid_map::Matrix& dataOutputMapAdd = outMap2["additional_layer"];

        outMap2["support_surface_smooth"] = dataOutputMapAdd + dataOutputSupSurfSmooth;

    }
    //footTipEmbeddingSimple();


    supportSurfaceUpperBounding(rawMap_, outMap2);

    //addSupportSurface(outMap2["support_surface_smooth"], outMap2["support_surface_added"]);



    // TEST: logically this makes sense!!
   // rawMap_["support_surface_smooth"] = outMap2["support_surface_smooth"];

  //  rawMap_["elevation_inpainted"] = inpaintedMap["elevation_inpainted"];
  //  rawMap_["elevation_smooth"] = inpaintedMap["elevation_smooth"];

  //  Eigen::Vector3f meanStance= getMeanStance();

  //  Position posCheck(meanStance(0), meanStance(1));

  //  Index indRaw;
  //  Index indInp;

  //  rawMap_.getIndex(posCheck,indRaw);
  //  inpaintedMap.getIndex(posCheck,indInp);

  //  std::cout << "Ind rawMap_: " << indRaw(0) << " " << indRaw(1) << std::endl;
  //  std::cout << "Ind rawMap_: " << indInp(0) << " " << indInp(1) << std::endl;


    // inpaintedMap.move(rawMap_.getPosition());

    //grid_map::Matrix& dataElev2 = rawMap_.get("elevation");
    //grid_map::Matrix dataElevSmoothedInpainted;

    //if(!filterChain2_.update(dataElev2, dataElevSmoothedInpainted));

    //inpaintedMap["elevation_smooth"] = dataElevSmoothedInpainted;


    // TODO: discriminate filled in


   // std::cout << inpaintedMap.getLayers()[0] << std::endl << std::endl << std::endl;
  //  std::cout << inpaintedMap.getLayers()[2] << std::endl;

   // rawMap_.get("elevation_inpainted") = dataElevInpainted - dataElev;


    addSupportSurface(outMap2);



    // Publish map
    grid_map_msgs::GridMap mapMessage;
    GridMapRosConverter::toMessage(outMap2, mapMessage);
    //mapMessage.info.header.frame_id = "/odom"; //! HACKED!!

    //GridMapRosConverter::fromMessage(mapMessage, rawMapCorrected)

    elevationMapInpaintedPublisher_.publish(mapMessage);




    //rawMap_.add("elevation_inpainted", dataElevInpainted);

    //inpaintFilter.update(dataElev, dataElevInpainted);


    // Here gaussian smoothing..


}

bool ElevationMap::terrainContinuityProcessing(){

    // Gaussian smoothing and hole filling operations..

}

//bool ElevationMAp::comparisonFootTipLayer(){
//    Position center(0.0, -0.15);
//     double radius = 0.4;
//
//      for (grid_map::CircleIterator iterator(map_, center, radius);
//          !iterator.isPastEnd(); ++iterator) {
//        map_.at("type", *iterator) = 1.0;
//        publish();
//        ros::Duration duration(0.02);
//        duration.sleep();
//      }
//}

bool ElevationMap::footTipEmbeddingSimple(){

}

Position3 ElevationMap::getFrontLeftFootTipPosition(){
    Position3 tipPos(frontLeftFootTip_(0), frontLeftFootTip_(1), frontLeftFootTip_(2));
    return tipPos;
}

Position3 ElevationMap::getFrontRightFootTipPosition(){
    Position3 tipPos(frontRightFootTip_(0), frontRightFootTip_(1), frontRightFootTip_(2));
    return tipPos;
}

double ElevationMap::getClosestMapValueUsingSpiralIterator(grid_map::GridMap& MapReference, Position footTip, double radius, double tipHeight){
    int counter = 0;
    for (grid_map::SpiralIterator iterator(MapReference, footTip, radius);
         !iterator.isPastEnd(); ++iterator) {   // Hacked to is inside..
        Index index(*iterator);
      //  std::cout << "Index 0: " << index(0) << " index 1: " << index(1) << std::endl;
        Position pos;
        MapReference.getPosition(index, pos);
      //  std::cout << "Check.." << std::endl;
        //if (MapReference.isValid(index)) std::cout << "It is valid" << std::endl;
        //else std::cout << "It is not valid" << std::endl;
        if (MapReference.isInside(pos) && !isnan(MapReference.at("support_surface_smooth", index)) && MapReference.isValid(index)){
       //     std::cout << "Found an isnan not ---------------------------------------------------------------------------------------" << std::endl;
            std::cout << "RETURNED DIFFERENCE TO A CLOSE NEIGHBOR USING SPIRALLING!!!" << std::endl;
            return tipHeight - MapReference.at("support_surface_smooth", index); // && MapReference.isValid(index)
        }
    //    std::cout << "got here" << std::endl;
        counter++;
        if (counter > 28) break;
    }
   // std::cout << " \n \n \n \n \n " << std::endl;
   // std::cout << "BEEN HERE< !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    return 0.0;
}

bool ElevationMap::supportSurfaceUpperBounding(GridMap& upperBoundMap, GridMap& supportSurfaceMap){

    Matrix& dataUpper = upperBoundMap["elevation"];
    Matrix& dataSup = supportSurfaceMap["support_surface_smooth"];

    supportSurfaceMap["support_surface_smooth"] = dataUpper.cwiseMin(dataSup);

    return true;
}

bool ElevationMap::addSupportSurface(GridMap& mapSmooth){


    // Get sensible central position upfront the robot.

    geometry_msgs::Transform footprint = getFootprint();

    std::cout << "FootPrint: " << footprint.translation.x << std::endl;
    std::cout << "FootPrint: " << footprint.translation.y << std::endl;
    std::cout << "FootPrint: " << footprint.translation.z << std::endl;
    //std::cout << "FootPrint: " << footprint.rotation. << std::endl;
    //std::cout << "FootPrint: " << footprint.translation.z << std::endl;
    //std::cout << "FootPrint: " << footprint.translation.z << std::endl;
    //std::cout << "FootPrint: " << footprint.translation.z << std::endl;


    tf::Quaternion quat;
    tf::quaternionMsgToTF(footprint.rotation, quat);

    tf::Vector3 footprintTrans;

    tf::vector3MsgToTF(footprint.translation, footprintTrans);

    tf::Matrix3x3 m(quat);

    tf::Vector3 trans({1.1,0.0,0.0});
    tf::Vector3 front({2.0,0.0,0.0});
    tf::Vector3 hind({0.2,0.0,0.0});
    tf::Vector3 right({1.1,0.9,0.0});
    tf::Vector3 left({1.1,-0.9,0.0});
    tf::Vector3 weightingReference({0.5, 0.0, 0.0});

    tf::Vector3 centerPoint = footprintTrans + m * trans;

    tf::Vector3 frontPoint = footprintTrans + m * front;
    tf::Vector3 hindPoint = footprintTrans + m * hind;
    tf::Vector3 rightPoint = footprintTrans + m * right;
    tf::Vector3 leftPoint = footprintTrans + m * left;
    tf::Vector3 weightingReferencePoint = footprintTrans + m * weightingReference;

    std::cout << "CenterPoint: " << centerPoint[0] << std::endl;
    std::cout << "CenterPoint: " << centerPoint[1] << std::endl;
    std::cout << "CenterPoint: " << centerPoint[2] << std::endl;


    // Color and shape definition of markers for foot tip ground contact visualization.
    visualization_msgs::Marker addingAreaMarkerList;
    addingAreaMarkerList.header.frame_id = "odom";
    addingAreaMarkerList.header.stamp = ros::Time();
    addingAreaMarkerList.ns = "elevation_mapping";
    addingAreaMarkerList.id = 0;
    addingAreaMarkerList.type = visualization_msgs::Marker::SPHERE_LIST;
    addingAreaMarkerList.action = visualization_msgs::Marker::ADD;
    addingAreaMarkerList.pose.orientation.x = 0.0;
    addingAreaMarkerList.pose.orientation.y = 0.0;
    addingAreaMarkerList.pose.orientation.z = 0.0;
    addingAreaMarkerList.pose.orientation.w = 1.0;
    addingAreaMarkerList.scale.x = 0.1;
    addingAreaMarkerList.scale.y = 0.1;
    addingAreaMarkerList.scale.z = 0.1;

    addingAreaMarkerList.color.a = 1.0; // Don't forget to set the alpha!
    addingAreaMarkerList.color.r = 1.0;
    addingAreaMarkerList.color.g = 0.1;
    addingAreaMarkerList.color.b = 0.1;

    geometry_msgs::Point p;
    p.x = centerPoint[0];
    p.y = centerPoint[1];
    p.z = centerPoint[2];
    addingAreaMarkerList.points.push_back(p);
    p.x = frontPoint[0];
    p.y = frontPoint[1];
    p.z = frontPoint[2];
    addingAreaMarkerList.points.push_back(p);
    p.x = hindPoint[0];
    p.y = hindPoint[1];
    p.z = hindPoint[2];
    addingAreaMarkerList.points.push_back(p);
    p.x = rightPoint[0];
    p.y = rightPoint[1];
    p.z = rightPoint[2];
    addingAreaMarkerList.points.push_back(p);
    p.x = leftPoint[0];
    p.y = leftPoint[1];
    p.z = leftPoint[2];
    addingAreaMarkerList.points.push_back(p);


    supportSurfaceAddingAreaPublisher_.publish(addingAreaMarkerList);
    // get central position in front of the robot and use circle iterator..
    // Simple testing: 0.8 * support surface new + 0.2 * support surface old.

    //supAdded = 0.5 * supAdded + 0.5 * supSmooth;
    // TODO: get submap of adequate size in front of the robot and add it to the supportSurface Added ..

    //rawMap_.getSubmap();

    Position center(centerPoint[0], centerPoint[1]);
    double radius = 0.9;

    // Circle Iterator!!!
    for (CircleIterator iterator(mapSmooth, center, radius); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        Position pos;
        rawMap_.getPosition(index, pos);
        double distanceToHind = sqrt(pow(pos(0) - weightingReferencePoint[0],2) + pow(pos(1) - weightingReferencePoint[1],2));
        double weight = 0.8 - 0.6 * distanceToHind / 1.8;
        rawMap_.at("support_surface_added", index) = (1-weight) * rawMap_.at("support_surface_added", index) + weight * mapSmooth.at("support_surface_smooth", index);
        //cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) << endl;
    }
}

bool ElevationMap::gaussianProcessModeling(){

}

bool ElevationMap::setFootprint(const geometry_msgs::Transform& footprint){
    footprint_ = footprint;
    return true;
}

geometry_msgs::Transform ElevationMap::getFootprint(){
    return footprint_;
}


} /* namespace */
