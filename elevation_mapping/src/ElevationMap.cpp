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
#include <elevation_mapping/PerformanceAssessment.h>

using namespace std;
using namespace grid_map;

namespace elevation_mapping {

ElevationMap::ElevationMap(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      hasUnderlyingMap_(false),
      visibilityCleanupDuration_(0.0)
{
    // Timon added foot_tip_elevation layer
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});



  clear();

  // TEST:
  elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);
  // END TEST

  // testing
  //rawMap_.setBasicLayers({"foot_tip_elevation"});


  // testing

  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
      nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
  // TODO if (enableVisibilityCleanup_) when parameter cleanup is ready.
  visbilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

  // Launching parameters.
  nodeHandle_.param("run_drift_adjustment", driftAdjustment_, true);
  nodeHandle_.param("apply_frame_correction", applyFrameCorrection_, true);
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
  initializeFootTipMarkers();

  // NEW: Publish data, for parameter tuning and visualization
  tuningPublisher1_ = nodeHandle_.advertise<elevation_mapping::PerformanceAssessment>("performance_assessment", 1000);

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
  driftEstimationPID_ = 0.0;
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
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "This is the thread ADD: " << this_id << std::endl;
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

  clean();
  rawMap_.setTimestamp(timestamp.toNSec()); // Point cloud stores time in microseconds.

  const ros::WallDuration duration = ros::WallTime::now() - methodStartTime;
  ROS_INFO("Raw map has been updated with a new point cloud in %f s.", duration.toSec());
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
  //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);
  // Set class variables.
  LFTipPostiion_(0) = (double)quadrupedState.contacts[0].position.x;
  LFTipPostiion_(1) = (double)quadrupedState.contacts[0].position.y;
  LFTipPostiion_(2) = (double)quadrupedState.contacts[0].position.z;
  RFTipPostiion_(0) = (double)quadrupedState.contacts[1].position.x;
  RFTipPostiion_(1) = (double)quadrupedState.contacts[1].position.y;
  RFTipPostiion_(2) = (double)quadrupedState.contacts[1].position.z;
  LFTipState_ = quadrupedState.contacts[0].state;
  RFTipState_ = quadrupedState.contacts[1].state;

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
    if(processStanceTriggerLeft_.size() > 1000){
        processStanceTriggerLeft_.erase(processStanceTriggerLeft_.begin());
    }
    if(processStanceTriggerRight_.size() > 1000){
        processStanceTriggerRight_.erase(processStanceTriggerRight_.begin());
    }

    // Collect the foot tip position data (if foot tip in contact). (Prevent nans)
    if(LFTipState_ && isInStanceLeft_){
        LFTipStance_.push_back(LFTipPostiion_);
    }
    if(RFTipState_ && isInStanceRight_){
        RFTipStance_.push_back(RFTipPostiion_);
    }

    // Stance Detection
    templateMatchingForStanceDetection("left", processStanceTriggerLeft_);
    templateMatchingForStanceDetection("right", processStanceTriggerRight_);

    return true;
}

bool ElevationMap::templateMatchingForStanceDetection(std::string tip, std::vector<bool> &stateVector)
{
    // Recognition of the end of a stance phase of the front legs.
    if(stateVector.size() >= 16 && stateVector.end()[-1]+stateVector.end()[-2]+
            stateVector.end()[-3]+stateVector.end()[-4]+stateVector.end()[-5]+
            stateVector.end()[-6]+stateVector.end()[-7]+stateVector.end()[-8] >= 6 &&
            stateVector.end()[-9]+stateVector.end()[-10] +
            stateVector.end()[-11]+stateVector.end()[-12]+ stateVector.end()[-13]+
            stateVector.end()[-14]+stateVector.end()[-15] <= 6){
        if(tip == "left" && !isInStanceLeft_){
            std::cout << "Start of LEFT stance" << std::endl;
            isInStanceLeft_ = 1;
        }
        if(tip == "right" && !isInStanceRight_){
            std::cout << "Start of Right stance" << std::endl;
            isInStanceRight_ = 1;
        }
    }

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
        }
        if(tip == "right" && isInStanceRight_){
            if(!processStance("right")) return false;
            isInStanceRight_ = 0;
        }
    }
    return true;
}

bool ElevationMap::processStance(std::string tip)
{
    std::cout << "Processing: " << tip <<std::endl;


    // Delete the last 10 entries of the Foot Stance Position Vector, as these are used for transition detection
    deleteLastEntriesOfStances(tip);
    getAverageFootTipPositions(tip);
    footTipElevationMapComparison(tip);
    publishAveragedFootTipPositionMarkers();
    // Performance Assessment, sensible if wanting to tune the system while walking on flat surface.
    performanceAssessmentMeanElevationMap();

    // Data Publisher for parameter tuning.
    tuningPublisher1_.publish(performance_assessment_msg_); // HACKED FOR DEBUGGING!!



    return true;
}

bool ElevationMap::deleteLastEntriesOfStances(std::string tip)
{
    // Delete the last entries of the stance, as these might be in the false state
    // (half the number of elements used in template matching are deleted)
    if (LFTipStance_.size() < 8){
        std::cout << "WARNING: LEFT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (RFTipStance_.size() < 8){
        std::cout << "WARNING: RIGHT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else{
        for (unsigned int i = 0; i < 8; ++i){
            if (tip == "left"){
                LFTipStance_.pop_back();
            }
            else if (tip == "right"){
                RFTipStance_.pop_back();
            }
        }
    }
    return true;
}

bool ElevationMap::getAverageFootTipPositions(std::string tip)
{
    Eigen::Vector3f totalStance(0, 0, 0);
    std::vector<Eigen::Vector3f> stance;
    if(tip == "left") stance = LFTipStance_;
    if(tip == "right") stance = RFTipStance_;
    LFTipStance_.clear();
    RFTipStance_.clear();

    // Derive mean foot tip positions
    if(stance.size() > 1){
        for (auto& n : stance){
            // Consider only those with state = 1  TODO!!!
            totalStance += n;
        }
        meanStance_ = totalStance / float(stance.size());
        processStanceTriggerLeft_.clear();
    }
    return true;
}

bool ElevationMap::publishAveragedFootTipPositionMarkers()
{
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

    // TESTING:
    std::cout << "xTip: " << xTip << std::endl;
    std::cout << "yTip: " << yTip << std::endl;
    // END TESTING

    // TODO: Clean nan supression scheme.
    bool useNewMethod = true;
    if(useNewMethod){

        // Offset of each step versus Elevation map.
        double verticalDifference = 0;
        // Weighted offset depending on the upper and lower bounds of the fused map.
        double weightedVerticalDifferenceIncrement = 0;

        // Horizontal position of the foot tip.
        Position tipPosition(xTip, yTip);

        // Make sure that the state is 1 and the foot tip is inside area covered by the elevation map.
        if(rawMap_.isInside(tipPosition)){ // HACKED FOR TESTS!!!
            float heightMapRaw = rawMap_.atPosition("elevation", tipPosition);
            float varianceMapRaw = rawMap_.atPosition("variance", tipPosition);
            float heightMapRawElevationCorrected = rawMap_.atPosition("elevation", tipPosition) + heightDifferenceFromComparison_; // Changed, since frame transformed grid map used.

            // Supress nans.
            if(!isnan(heightMapRaw) && !isnan(heightDifferenceFromComparison_)){

                // Calculate difference.
                verticalDifference = zTip - heightMapRaw;
                double verticalDifferenceCorrected = zTip - heightMapRawElevationCorrected;
                std::cout << "HeightDifference CLASSICAL:    " << verticalDifference << "        HeightDifference CORRECTED:    " << verticalDifferenceCorrected << std::endl;

                // New Tuning piblisher.

                performance_assessment_msg_.first_measure = verticalDifferenceCorrected;

                // Wait some stances to get sorted at the beginning.
                if(weightedDifferenceVector_.size() >= 4) performanceAssessment_ += fabs(verticalDifferenceCorrected);
                std::cout << "Cumulative Performance Value (summed up weighted differences): " << performanceAssessment_ << std::endl;

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
                std::cout << "lower: " << lowerBoundFused << " elev: " << elevationFused << " upper: " << upperBoundFused << std::endl;
                weightedVerticalDifferenceIncrement = gaussianWeightedDifferenceIncrement(lowerBoundFused, elevationFused, upperBoundFused, verticalDifference);

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


                // Uses the footContactMarkerList_, therefore called here.
                // Updates the map layer, that purely relies on the last n foot tips.
                int numberOfConsideredFootTips = 10;

                //! HACKED AWAY, no foot tip layer updates now!!
                //updateFootTipBasedElevationMapLayer(numberOfConsideredFootTips);

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

                // Store the vertical difference history in a vector for PID controlling.
                weightedDifferenceVector_.push_back(heightDifferenceFromComparison_ + weightedVerticalDifferenceIncrement); // TODO: check viability
                if(weightedDifferenceVector_.size() > 6) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
                std::cout << "Weighted Height Difference: " << weightedDifferenceVector_[0] << "\n";

                // Longer weightedDifferenceVector for PID drift calculation
                if(!isnan(heightDifferenceFromComparison_)) PIDWeightedDifferenceVector_.push_back(heightDifferenceFromComparison_); // Removed the vertical difference increment, TEST IT!
                if(PIDWeightedDifferenceVector_.size() >= 30) PIDWeightedDifferenceVector_.erase(PIDWeightedDifferenceVector_.begin());


                // New: create one for the left and one for the right foot tips, to get separate drift estimation
                //if (tip == "right")

                // TODO: for Kalman drift estimation, and PID with left and right separated...

                // PID height offset calculation.
                double heightDiffPID = differenceCalculationUsingPID();




                // Avoid Old Nans to be included. TEST!! // TODO: check where nans might come from to avoid them earlier
                //if(!isnan(heightDiffPID))

                // PID based drift estimation.
                driftEstimationPID_ = driftCalculationUsingPID(tip);
                std::cout << "Size of left tip stance: " << LFTipStance_.size() << "\n";
                std::cout << "Size of right tip stance: " << RFTipStance_.size() << "\n";


                if (isnan(driftEstimationPID_)){
                    std::cout << "NULLED THE DRIFT FOR NAN REASONS!! \n";
                    driftEstimationPID_ = 0.0;
                }

                double driftEstimationPIDadder = 0;
                if (heightDiffPID != 0.0) driftEstimationPIDadder = driftEstimationPID_ * (heightDiffPID/fabs(heightDiffPID));
                else driftEstimationPIDadder = driftEstimationPID_;

                // TESTING, adjusted this, check consequences..  // HACKED ALSO IN IF ARGUMENT!!! // Hacked from minus to plus!!!!
                if(applyFrameCorrection_) heightDifferenceFromComparison_ = heightDiffPID + driftEstimationPIDadder;// - driftEstimationPIDadder;//  driftEstimationPID_; //! HACKED FOR TESTING< ABSOLUTELY TAKE OUT AGAIN!!!
                else heightDifferenceFromComparison_ = 0.0;
                //if(applyFrameCorrection_ && !footTipOutsideBounds_)

                // For Tuning.
                performance_assessment_msg_.third_measure = heightDifferenceFromComparison_;

                // Kalman Filter based offset calculation.
                //auto diffTuple = differenceCalculationUsingKalmanFilter();
                //estimatedKalmanDiffIncrement_ =  std::get<0>(diffTuple);
                //estimatedKalmanDiff_ += estimatedKalmanDiffIncrement_;
                //PEstimatedKalmanDiffIncrement_ = std::get<1>(diffTuple);
                //std::cout << "Kalman Filtered Difference Estimate!" << estimatedKalmanDiff_ << "VS. PID Difference: " << heightDifferenceFromComparison_ << std::endl;
                // TESTING:
                //heightDifferenceFromComparison_ = estimatedKalmanDiff_;



                performance_assessment_msg_.fifth_measure = driftEstimationPID_;

                double second_measure_factor = -10000.0;
                performance_assessment_msg_.second_measure = driftEstimationPID_ * weightedDifferenceVector_[5] * second_measure_factor;
                std::cout << "weightedDifferenceVector!!: LAst element: " << weightedDifferenceVector_[5] << " Drift Est PID!!!:::::: " << (double)driftEstimationPID_ << std::endl;


                // TODO: Add some low pass filtering: FUTURE: Low passing for Mode changing..
                // Low pass filtering for robust high grass detection ****************************************************
                if (grassDetectionHistory_.size() > 2) grassDetectionHistory_.erase(grassDetectionHistory_.begin());
                if (performance_assessment_msg_.second_measure > 0.1){
                    std::cout << "ADDED A 1 TO HIGH GRASS FILTERING!!" << std::endl;
                    grassDetectionHistory_.push_back(1);
                }
                else grassDetectionHistory_.push_back(0);

                int sum_of_elems = 0;
                for (bool n : grassDetectionHistory_)
                    sum_of_elems += n;
                if (sum_of_elems > 0)
                    std::cout << "HIGH GRASS MODE ACTIVATED!!!!!" << "\n" << "\n" << "\n" << "\n" << "\n" << std::endl;
                // End low pass filtering  ************************************************



            }
            else{
                //if(!isnan(driftEstimationPID_)) heightDifferenceFromComparison_ += driftEstimationPID_; // HACKED AWAY FOR TESTING!
                std::cout << "heightDifferenceFromComparison_ was Incremented by estimated Drift because NAN was found: " << (double)driftEstimationPID_ << std::endl;
            }
        }
        else std::cout << "heightDifferenceFromComparison_ wasnt changed because tip is not inside map area" << std::endl;
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
    odomMapTransform.getOrigin()[2] += heightDifferenceFromComparison_;

    //ros::Time stamp = ros::Time().fromNSec(fusedMap_.getTimestamp());

    //std::cout << "TIMESTAMP PUBLISHED THE odom_drift_adjusted TRANSFORM!!: " << stamp << std::endl;

    mapCorrectedOdomTransformBroadcaster_.sendTransform(tf::StampedTransform(odomMapTransform,
                                          ros::Time().fromNSec(rawMap_.getTimestamp()), "odom", "odom_drift_adjusted"));

    return true;
}

float ElevationMap::differenceCalculationUsingPID()
{
    // TODO: Tune these, they are only guessed so far.
    float kp = 0.24;
    float ki = 0.76;
    float kd = -0.07; // Trying PI controller for now.. (Quite good between 0 and -0.1)

    // Nan prevention.
    if(weightedDifferenceVector_.size() > 1 && isnan(weightedDifferenceVector_[1])) return 0.0;
    else if(weightedDifferenceVector_.size() > 1){
    // Calculate new total diff here
    double totalDiff = 0.0;
    for (auto& n : weightedDifferenceVector_)
        totalDiff += n;
    double meanDiff = totalDiff / float(weightedDifferenceVector_.size());
    return kp * weightedDifferenceVector_[weightedDifferenceVector_.size()] + ki * meanDiff +
            kd * (weightedDifferenceVector_[weightedDifferenceVector_.size()] - weightedDifferenceVector_[weightedDifferenceVector_.size()-1]);
    }
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
        std::cout << "ATTENTIONATTENTION \n" << "******* \n" << "********* \n";
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


    // New Stuff for testing!!
    //! For testing
    if(elevation + diff < lowerBound + 0.5 * (elevation - lowerBound)){
        std::cout << "************************************* SOFT LOWER!! **************************!!!!!!!!!!!!!!!!!!!!" << std::endl;
        applyFrameCorrection_ = false; // Hacked!!!!
        grassDetectionHistory2_.push_back(1);
    }
    else{
        applyFrameCorrection_ = true;
        grassDetectionHistory2_.push_back(0);
    }
    if (grassDetectionHistory2_.size() > 8) grassDetectionHistory2_.erase(grassDetectionHistory2_.begin());

    // TODO: At some point create a triger fct, that does this..
    int triggerSum = 0;
    for (auto n : grassDetectionHistory2_){
        triggerSum += n;
    }

    // DEBugging:
    if (triggerSum > 1){
        std::cout << "<<<<<<<<>>>>>>>>> \n" << "<<<<<<<<<<>>>>>>>> \n" << "<<<<<<<<<<>>>>>>>>> \n";
    }
    else{
        std::cout << "!!!! /n" << "!!!! \n" << "!!!! \n";
    }

    // End New *******************************************************************************************************************


    // Error difference weighted as (1 - gaussian), s.t. intervall from elevation map to bound contains two standard deviations
    double weight = 0.0;
    float standardDeviationFactor = 1;  //! THIS MAY BE A LEARNING FUNCTION IN FUTURE!! // CHANGED
    if(diff < 0.0){
        weight = normalDistribution(diff, 0.0, fabs(elevation-lowerBound)*standardDeviationFactor);





        //! EndTesting

        // For Coloration
        if(elevation + diff < lowerBound){

            std::cout << "******************************************** LOWER ******************************" << std::endl;
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

    std::cout << "WEIGHT: " << (1-weight) << " diff: " << diff << " elevation: " << elevation <<
                 " lower: " << lowerBound << " upper: " << upperBound << std::endl;
    return (1.0 - weight) * diff;
}

std::tuple<double, double> ElevationMap::differenceCalculationUsingKalmanFilter()
{
    double predDiff, PPredDiff, measDiff, PMeasDiff;
    if(weightedDifferenceVector_.size() > 0 && !isnan(weightedDifferenceVector_[0])){

      //  measurement = newheightdiff - oldheightdiff
        double diffMeasurement = weightedDifferenceVector_[0] - estimatedKalmanDiff_;
        std::cout << "WeightedDifferenceVector inside Kalman Filter: " << weightedDifferenceVector_[0] << std::endl;
        float R = 0.95;
        float Q = 0.05;
    //    // Prediction Step.
        predDiff = estimatedKalmanDiffIncrement_;
        PPredDiff = PEstimatedKalmanDiffIncrement_ + Q;

    //    // Measurement Step.
        double K = PPredDiff / (PPredDiff+R);
        measDiff = predDiff + K * diffMeasurement;
        PMeasDiff = PPredDiff - K * PPredDiff;

        std::cout << "mean measKalmanDiff: " << measDiff << std::endl;
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
    //std::cout << "points inside updateFootTipBased...: " << footContactMarkerList_.points.back().x << " " << footContactMarkerList_.points.back().y <<
    //             " " << footContactMarkerList_.points.back().z << std::endl;
    //std::cout << "Layers!!: " << rawMap_.getLayers()[11] << std::endl;

    rawMap_.add("foot_tip_elevation", 0);

    double summedErrorValue = 0;
    for (GridMapIterator iterator(rawMap_); !iterator.isPastEnd(); ++iterator) {

        //Position3 posMap;
        //rawMap_.getPosition3("foot_tip_elevation", *iterator, posMap);
        Position posMap;
        rawMap_.getPosition(*iterator, posMap);
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

        // Calculate Difference measure in restricted area around foot tips (the last two)
        double threshold = 0.4;
        if (rawMap_.isInside(posMap) && !isnan(rawMap_.atPosition("elevation", posMap)) && distanceVector.size() > 1){
            if (distanceVector[0] < threshold || distanceVector[1] < threshold)
                if (rawMap_.isInside(posMap)) rawMap_.atPosition("foot_tip_elevation", posMap) = cellHeight; // Hacked to only update restricted area around foot tips..
                summedErrorValue += fabs(cellHeight - (rawMap_.atPosition("elevation", posMap)+heightDifferenceFromComparison_)); // Hacked a little error to check sensitivity
            // Consider adding the offset, as the non moved one may be considered..
            //std::cout << "Difference in height, what about offset?: " << fabs(cellHeight - rawMap_.atPosition("elevation", posMap)) << std::endl;
        }
    }
    std::cout << "Total ERROR VALUE OF FOOT TIP ELVATION MAP VS CAMERA ELEVATION MAP: " << summedErrorValue << std::endl;
    return true;
}

bool ElevationMap::performanceAssessmentMeanElevationMap()
{
    // Choose area to cut out (to be left aside for performance assessment, i.e. nonplanar obstacles)
    double xMinCutoff, xMaxCutoff, yMinCutoff, yMaxCutoff;
    xMinCutoff = yMinCutoff = -100000000000;
    xMaxCutoff = yMaxCutoff = 100000000000;

    // Set them.
    xMinCutoff = 1.3;
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
    std::cout << "MEAN:                                " << totalElevationMapHeight / double(counter) << std::endl;
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

    std::cout << "PERFORMANCE ASSESSMENT MEAN:                                " << performanceAssessment/counter << std::endl;

    // Add a factor for scaling in rqt plot
    double factor = 200.0;

    performance_assessment_msg_.fourth_measure = factor * performanceAssessment/counter;

}

} /* namespace */
