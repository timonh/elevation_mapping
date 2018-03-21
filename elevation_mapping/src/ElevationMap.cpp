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
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  // TEST
  rawMap_.add("elevation_corrected");
  // END TEST

  clear();


  // TEST:
  elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);
  // END TEST
  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
      nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
  // TODO if (enableVisibilityCleanup_) when parameter cleanup is ready.
  visbilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

  nodeHandle_.param("drift_adjustment", driftAdjustment_, true);
  if(driftAdjustment_){
  // (New:) Foot tip position Subscriber for Foot tip - Elevation comparison
      bool use_bag = true;
      if(!use_bag) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &ElevationMap::footTipStanceCallback, this);
      else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 1, &ElevationMap::footTipStanceCallback, this);
  }

  // NEW: publish foot tip markers
  footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000);
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("elevation_map_bound_markers_rviz", 1000);
  initializeFootTipMarkers();

  // Class Variable to model the compared difference evolution.
  oldDiff_ = 0.0;
  oldDiffPID_ = 0.0;
  estimatedDrift_ = 0.0;
  estimatedDriftVariance_ = 0.0;
  // Some Parameters.
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

  // Initialization for time calculation.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

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

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength,
                       requestedIndexInSubmap, position, length, rawMap_.getLength(),
                       rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getSize(),
                       rawMap_.getStartIndex());

  return fuse(topLeftIndex, submapBufferSize);
}

bool ElevationMap::fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size)
{
  ROS_DEBUG("Fusing elevation map...");

  // Nothing to do.
  if ((size == 0).any()) return false;

  // Initializations.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  //! ATTENTION!!! REMOVED THIS SCOPED LOCK!! CHECK WHAT THE CONSEQUENCE IS..
  // Copy raw elevation map data for safe multi-threading.
  //boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  auto& rawMapCopy = rawMap_; // HACKING AROUND!!!
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
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
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
  //! DEBUG:
  std::cout << "FOR SOME REASON PUBLISHING THE FUSED MAP" << std::endl;
  //! END DEBUG
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

bool ElevationMap::publishSpatialVariancePointCloud(Eigen::VectorXf& spatialVariances)
{
  int size = spatialVariances.size();
  ROS_INFO("VariancePointCloudPublisherfct..");
  std::cout << "in: " << size << std::endl;
  for(unsigned int i = 1; i <= spatialVariances.SizeAtCompileTime; ++i ){
      const float& spatialpointvariance = spatialVariances(i);

      //std::cout << typeid(spatialpointvariance).name() << std::endl;
      //std::cout << (double)spatialpointvariance << std::endl;
      //ROS_INFO("This one: %f", spatialpointvariance);
  }

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
}

bool ElevationMap::detectStancePhase()
{

    boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);

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

    // TODO: Create Template Matching function, wich saves coding lines
    templateMatchingForStanceDetection("left", processStanceTriggerLeft_);
    templateMatchingForStanceDetection("right", processStanceTriggerRight_);


//    // Recognition of the end of a stance phase of the front legs.
//    if(processStanceTriggerLeft_.size() >= 20 && processStanceTriggerLeft_.end()[-1]+processStanceTriggerLeft_.end()[-2]+
//            processStanceTriggerLeft_.end()[-3]+processStanceTriggerLeft_.end()[-4]+processStanceTriggerLeft_.end()[-5]+
//            processStanceTriggerLeft_.end()[-6]+processStanceTriggerLeft_.end()[-7]+processStanceTriggerLeft_.end()[-8]+
//            processStanceTriggerLeft_.end()[-9]+processStanceTriggerLeft_.end()[-10] >= 8 &&
//            processStanceTriggerLeft_.end()[-11]+processStanceTriggerLeft_.end()[-12]+ processStanceTriggerLeft_.end()[-13]+
//            processStanceTriggerLeft_.end()[-14]+processStanceTriggerLeft_.end()[-15]+processStanceTriggerLeft_.end()[-16] +
//            processStanceTriggerLeft_.end()[-17]+processStanceTriggerLeft_.end()[-18]+processStanceTriggerLeft_.end()[-19] <= 10 &&
//            !isInStanceLeft_){
//        std::cout << "Start of LEFT stance" << std::endl;
//        isInStanceLeft_ = 1;
//    }

//    if(processStanceTriggerRight_.size() >= 20 && processStanceTriggerRight_.end()[-1]+processStanceTriggerRight_.end()[-2]+
//            processStanceTriggerRight_.end()[-3]+processStanceTriggerRight_.end()[-4]+processStanceTriggerRight_.end()[-5] +
//            processStanceTriggerRight_.end()[-6]+processStanceTriggerRight_.end()[-7]+processStanceTriggerRight_.end()[-8]+
//            processStanceTriggerRight_.end()[-9]+processStanceTriggerRight_.end()[-10] >= 8 &&
//            processStanceTriggerRight_.end()[-11]+processStanceTriggerRight_.end()[-12]+processStanceTriggerRight_.end()[-13]+
//            processStanceTriggerRight_.end()[-14]+processStanceTriggerRight_.end()[-15]+processStanceTriggerRight_.end()[-16]+
//            processStanceTriggerRight_.end()[-17]+processStanceTriggerRight_.end()[-18]+processStanceTriggerRight_.end()[-19] <= 10 &&
//            !isInStanceRight_){
//        std::cout << "Start of RIGHT stance" << std::endl;
//        isInStanceRight_ = 1;
//    }

//    // Recognition of the start of a stance phase of the front legs.
//    if(processStanceTriggerLeft_.size() >= 20 &&
//            processStanceTriggerLeft_.end()[-1] + processStanceTriggerLeft_.end()[-2] + processStanceTriggerLeft_.end()[-3]+
//            processStanceTriggerLeft_.end()[-4]+processStanceTriggerLeft_.end()[-5] + processStanceTriggerLeft_.end()[-6]+
//            processStanceTriggerLeft_.end()[-7]+processStanceTriggerLeft_.end()[-8]+
//            processStanceTriggerLeft_.end()[-9]+processStanceTriggerLeft_.end()[-10] <= 3 &&
//            processStanceTriggerLeft_.end()[-11]+processStanceTriggerLeft_.end()[-12]+ processStanceTriggerLeft_.end()[-13]+
//            processStanceTriggerLeft_.end()[-14]+processStanceTriggerLeft_.end()[-15]+processStanceTriggerLeft_.end()[-16] +
//            processStanceTriggerLeft_.end()[-17]+processStanceTriggerLeft_.end()[-18]+processStanceTriggerLeft_.end()[-19] >=1 &&
//            isInStanceLeft_){
//        if(!processStance("Left")) return false;
//        isInStanceLeft_ = 0;
//    }

//    // Recognition of the start of a stance phase of the front legs.
//    if(processStanceTriggerRight_.size() >= 20 &&
//            processStanceTriggerRight_.end()[-1] + processStanceTriggerRight_.end()[-2] + processStanceTriggerRight_.end()[-3]+
//            processStanceTriggerRight_.end()[-4]+processStanceTriggerRight_.end()[-5] + processStanceTriggerRight_.end()[-6]+
//            processStanceTriggerRight_.end()[-7]+processStanceTriggerRight_.end()[-8]+
//            processStanceTriggerRight_.end()[-9]+processStanceTriggerRight_.end()[-10] <= 3 &&
//            processStanceTriggerRight_.end()[-11]+processStanceTriggerRight_.end()[-12]+processStanceTriggerRight_.end()[-13]+
//            processStanceTriggerRight_.end()[-14]+processStanceTriggerRight_.end()[-15]+processStanceTriggerRight_.end()[-16]+
//            processStanceTriggerRight_.end()[-17]+processStanceTriggerRight_.end()[-18]+processStanceTriggerRight_.end()[-19] >=1 &&
//            isInStanceRight_){
//        if(!processStance("Right")) return false;
//        isInStanceRight_ = 0;
//    }

    return true;
}

bool ElevationMap::templateMatchingForStanceDetection(std::string tip, std::vector<bool> &stateVector)
{
    //std::cout << "TIP TYPE: " << tip << " state vec size: " << stateVector.size() << "\n";

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
    publishAveragedFootTipPositionMarkers();
    footTipElevationMapComparison(tip);

//    // Delete the last 10 entries of the Foot Stance Position Vector, as these are used for transition detection
//    if(LFTipStance_.size() > 10 && tip == "Left"){
//        for(unsigned int i = 0; i < 10; ++i){
//            LFTipStance_.pop_back();
//        }
//    }
//    else if(tip == "Left"){
//        std::cout << "WARNING: LEFT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
//        return false;
//    }

//    // Delete the last 10 entries of the Foot Stance Position Vector, as these are used for transition detection
//    if(RFTipStance_.size() > 10 && tip == "Right"){
//        for(unsigned int i = 0; i < 10; ++i){
//            RFTipStance_.pop_back();
//        }
//    }
//    else if(tip == "Right"){
//        std::cout << "WARNING: RIGHT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
//        return false;
//    }


    // TODO: Averaging function here!!


//    double xTotalLeft = 0.0;
//    double yTotalLeft = 0.0;
//    double zTotalLeft = 0.0;
//    double xTotalRight = 0.0;
//    double yTotalRight = 0.0;
//    double zTotalRight = 0.0;


//    // For publisher of mean foot tip positions
//    if(tip == "left" && LFTipStance_.size() > 1){

//        geometry_msgs::Point p;
//        for (auto& n : LFTipStance_){
//            // Consider only those with state = 1
//            xTotalLeft += n(0);
//            yTotalLeft += n(1);
//            zTotalLeft += n(2);
//        }

//        xMeanStanceLeft_ = xTotalLeft / float(LFTipStance_.size());
//        yMeanStanceLeft_ = yTotalLeft / float(LFTipStance_.size());
//        zMeanStanceLeft_ = zTotalLeft / float(LFTipStance_.size());

//        std::cout << "x Mean Left: " << xMeanStanceLeft_ << "Stance size: " << LFTipStance_.size() << std::endl;
//        std::cout << "y Mean Left: " << yMeanStanceLeft_ << "Stance size: " << LFTipStance_.size() << std::endl;
//        std::cout << "z Mean Left: " << zMeanStanceLeft_ << "Stance size: " << LFTipStance_.size() << std::endl;

//        //std::cout << "LSIZE: " << LFTipStance_.size() << std::endl;
//        LFTipStance_.clear();
//        processStanceTriggerLeft_.clear();
//        //std::cout << "LSIZE: " << LFTipStance_.size() << std::endl;


//        // TODO: Foot Tip Publisher function here!!


//        // Positions for publisher
//        p.x = xMeanStanceLeft_;
//        p.y = yMeanStanceLeft_;
//        p.z = zMeanStanceLeft_;

//        // Check for nans
//        if(p.x != p.x || p.y != p.y || p.z != p.z){
//            std::cout << "NAN FOUND!!" << std::endl;
//        }
//        else footContactMarkerList_.points.push_back(p);
//    }

//    if(tip == "right" && RFTipStance_.size() > 1){

//        geometry_msgs::Point p;
//        for (auto& n : RFTipStance_){
//            // Consider only those with state = 1
//            xTotalRight += n(0);
//            yTotalRight += n(1);
//            zTotalRight += n(2);
//        }
//        xMeanStanceRight_ = xTotalRight / float(RFTipStance_.size());
//        yMeanStanceRight_ = yTotalRight / float(RFTipStance_.size());
//        zMeanStanceRight_ = zTotalRight / float(RFTipStance_.size());


//        std::cout << "x Mean Right: " << xMeanStanceRight_ << "Stance Size: " << RFTipStance_.size() << std::endl;
//        std::cout << "y Mean Right: " << yMeanStanceRight_ << "Stance Size: " << RFTipStance_.size() << std::endl;
//        std::cout << "z Mean Right: " << zMeanStanceRight_ << "Stance Size: " << RFTipStance_.size() << std::endl;
//        std::cout << "z total Right: " << zTotalRight << std::endl;

//        //std::cout << "RSIZE: " << RFTipStance_.size() << std::endl;
//        RFTipStance_.clear();
//        processStanceTriggerRight_.clear();
//        //std::cout << "RSIZE: " << RFTipStance_.size() << std::endl;

//        // Positions for publisher
//        p.x = xMeanStanceRight_;
//        p.y = yMeanStanceRight_;
//        p.z = zMeanStanceRight_;

//        // Check for nans
//        if(p.x != p.x || p.y != p.y || p.z != p.z){
//            std::cout << "NAN FOUND!!" << std::endl;
//        }
//        else footContactMarkerList_.points.push_back(p);
//    }

    return true;
}

bool ElevationMap::deleteLastEntriesOfStances(std::string tip)
{

    if(LFTipStance_.size() < 10){
        std::cout << "WARNING: LEFT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if(RFTipStance_.size() < 10){
        std::cout << "WARNING: RIGHT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else{
        for(unsigned int i = 0; i < 10; ++i){
            if(tip == "left"){
                LFTipStance_.pop_back();
            }
            else if(tip == "right"){
                RFTipStance_.pop_back();
            }
        }
    }

    return true;
}

bool ElevationMap::getAverageFootTipPositions(std::string tip)
{
    Eigen::Vector3f totalStance(0, 0, 0);

    // TODO: check if this is valid!
    std::vector<Eigen::Vector3f> stance;
    if(tip == "left") stance = LFTipStance_;
    if(tip == "right") stance = RFTipStance_;

    // For publisher of mean foot tip positions
    if(stance.size() > 1){

        geometry_msgs::Point p;
        for (auto& n : stance){
            // Consider only those with state = 1  TODO!!!
            totalStance += n;
        }

        if(tip == "left"){
            meanStanceLeft_ = totalStance / float(stance.size());
            LFTipStance_.clear();
            processStanceTriggerLeft_.clear();
            // Positions for publisher
            p.x = meanStanceLeft_(0);
            p.y = meanStanceLeft_(1);
            p.z = meanStanceLeft_(2);
        }
        if(tip == "right"){
            meanStanceRight_ = totalStance / float(stance.size());
            RFTipStance_.clear();
            processStanceTriggerRight_.clear();
            // Positions for publisher
            p.x = meanStanceRight_(0);
            p.y = meanStanceRight_(1);
            p.z = meanStanceRight_(2);
        }

        // Check for nans
        if(p.x != p.x || p.y != p.y || p.z != p.z){
            std::cout << "NAN FOUND IN MEAN FOOT TIP POSITION!!" << std::endl;
        }
        else footContactMarkerList_.points.push_back(p);
    }
    return true;
}

bool ElevationMap::publishAveragedFootTipPositionMarkers()
{
    // Publish averaged foot tip positions
    footContactPublisher_.publish(footContactMarkerList_);
    return true;
}


bool ElevationMap::footTipElevationMapComparison(std::string tip)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceComparisonMutex_);
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);

    // New version
    double xTip, yTip, zTip;
    if(tip == "left"){
        xTip = meanStanceLeft_(0);
        yTip = meanStanceLeft_(1);
        zTip = meanStanceLeft_(2);
    }
    else if(tip == "right"){
        xTip = meanStanceRight_(0);
        yTip = meanStanceRight_(1);
        zTip = meanStanceRight_(2);
    }

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
        if(rawMap_.isInside(tipPosition)){
            float heightMapRaw = rawMap_.atPosition("elevation", tipPosition);
            float varianceMapRaw = rawMap_.atPosition("variance", tipPosition);
            float heightMapRawElevationCorrected = rawMap_.atPosition("elevation", tipPosition) + heightDifferenceFromComparison_; // Changed, since frame transformed grid map used.

            // Supress nans.
            if(!isnan(heightMapRaw)){

                // Calculate difference.
                verticalDifference = zTip - heightMapRaw;
                double verticalDifferenceCorrected = zTip - heightMapRawElevationCorrected;
                std::cout << "HeightDifference CLASSICAL:    " << verticalDifference << "        HeightDifference CORRECTED:    " << verticalDifferenceCorrected << std::endl;

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

                // Store the vertical difference history in a vector for PID controlling.
                weightedDifferenceVector_.push_back(heightDifferenceFromComparison_ + weightedVerticalDifferenceIncrement); // TODO: check viability
                if(weightedDifferenceVector_.size() >= 6) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
                std::cout << "Weighted Height Difference: " << weightedDifferenceVector_[0] << "\n";

                // PID height offset calculation.
                double heightDiffPID = differenceCalculationUsingPID();
                heightDifferenceFromComparison_ = heightDiffPID;
            }
            else std::cout << "heightDifferenceFromComparison_ wasnt changed because NAN was found" << std::endl;
        }
        else std::cout << "heightDifferenceFromComparison_ wasnt changed because tip is not inside map area" << std::endl;
    }

    // Publish frame, offset by the height difference parameter.
    frameCorrection();

    // Publish the elevation map with the new layer, at the frequency of the stances.
    grid_map_msgs::GridMap mapMessage;
    GridMapRosConverter::toMessage(rawMap_, mapMessage);
    mapMessage.info.header.frame_id = "odom_drift_adjusted"; //! HACKED!!

    //! Now this has to be done here if Data is to be used: Disscuss with Lorenz if this is more efficient than the additional Layer?
    //GridMapRosConverter::fromMessage(mapMessage, rawMapCorrected)

    elevationMapCorrectedPublisher_.publish(mapMessage);

//    // Offset of each step versus Elevation map.
//    double diff = 0;
//    double weightedDifference = 0;

//    // Test:
//    bool fastComparison = true;
//    if(fastComparison){
//        if(LFTipState_ == 1){
//            Position posLeft(xLeft, yLeft);

//            // Only if rawMap_ has entries.
//            if(rawMap_.getSize()(0) > 1){

//                float heightLeft, varLeft, heightLeftOffsetCorrected;
//                std::cout << "Just before LEFT Map comparison" << std::endl;
//                if(rawMap_.isInside(posLeft)){
//                    heightLeft = rawMap_.atPosition("elevation", posLeft);
//                    varLeft = rawMap_.atPosition("variance", posLeft);
//                    heightLeftOffsetCorrected = rawMap_.atPosition("elevation_corrected", posLeft);

//                    diff = zLeft - heightLeft;
//                    double diffCorrected = zLeft - heightLeftOffsetCorrected;
//                    std::cout << "HeightDifference classical: " << diff << "HeightDifference corrected: " << diffCorrected << std::endl;


//                    // Fusion for robust error calculation.
//                    Eigen::Array2d length;
//                    if(rawMap_.isInside(posLeft) && !std::isnan(rawMap_.atPosition("horizontal_variance_x",posLeft))){
//                        std::cout << "Size of fused area: " << rawMap_.atPosition("horizontal_variance_x",posLeft) << " and: " << rawMap_.atPosition("horizontal_variance_y",posLeft) << std::endl;

//                        // Use 3 standard deviations in each direction for robustness of fusion
//                        length[0] = std::max(6 * sqrt(rawMap_.atPosition("horizontal_variance_x",posLeft)), 6 * sqrt(rawMap_.atPosition("horizontal_variance_y",posLeft)));
//                        length[1] = length[0];
//                        // HACKED!!!
//                        auto boundTuple = getFusedCellBounds(posLeft, length);
//                        std::cout << "lower: " << std::get<0>(boundTuple) << " elev: " << std::get<1>(boundTuple) << " upper: " << std::get<2>(boundTuple) << std::endl;
//                        double lower = std::get<0>(boundTuple);
//                        double elev = std::get<1>(boundTuple);
//                        double upper = std::get<2>(boundTuple);
//                        weightedDifference = gaussianWeightedDifferenceIncrement(lower, elev, upper, diff);
//                        // weighted difference history stored in a vector for PID calculation
//                        weightedDifferenceVector_.push_back(heightDifferenceFromComparison_ + weightedDifference); // TODO: check viability
//                        if(weightedDifferenceVector_.size() >= 6) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
//                        std::cout << "Weighted Height Difference: " << weightedDifference << "\n";
//                    }

//                    // Preparing integral part for PID
//                    if(fabs(weightedDifference) < 10.0){
//                        totalHeightDifference_ += weightedDifference;
//                        heightDifferenceComponentCounter_ += 1;
//                    }

//                    // Filter for drift estimation.
//                    estimatedDrift_, estimatedDriftVariance_ = filteredDriftEstimation(weightedDifference, estimatedDrift_, estimatedDriftVariance_);
//                }
//                else std::cout << "posLEft is outside of range!" << std::endl;
//            }
//        }
//        //else std::cout << "LEFT LIFTED!!!" << std::endl;

//        if(RFTipState_ == 1){
//            Position posRight(xRight, yRight);

//            // Only if rawMap_ has entries.
//            if(rawMap_.getSize()(0) > 1){

//                std::cout << "Just before RIGHT Map comparison" << std::endl;

//                float heightRight, varRight, horVarRight, heightRightOffsetCorrected;
//                if(rawMap_.isInside(posRight)){
//                    heightRight = rawMap_.atPosition("elevation", posRight);
//                    varRight = rawMap_.atPosition("variance", posRight);
//                    horVarRight = rawMap_.atPosition("horizontal_variance_x", posRight);
//                    heightRightOffsetCorrected = rawMap_.atPosition("elevation_corrected", posRight);

//                    std::cout << "Just after RIGHT Map comparison" << std::endl;

//                    diff = zRight - heightRight;
//                    double diffCorrected = zRight - heightRightOffsetCorrected;
//                    std::cout << "HeightDifference classical: " << diff << "HeightDifference corrected: " << diffCorrected << std::endl;


//                    // Fusion for robust error calculation.
//                    Eigen::Array2d length;
//                    if(rawMap_.isInside(posRight) && !std::isnan(rawMap_.atPosition("horizontal_variance_x",posRight))){
//                        std::cout << "Size of fused area: " << rawMap_.atPosition("horizontal_variance_x",posRight) << " and: " << rawMap_.atPosition("horizontal_variance_y",posRight) << std::endl;
//                        length[0] = std::max(6 * sqrt(rawMap_.atPosition("horizontal_variance_x",posRight)), 6 * sqrt(rawMap_.atPosition("horizontal_variance_y",posRight)));
//                        length[1] = length[0];

//                        auto boundTuple = getFusedCellBounds(posRight, length);
//                        std::cout << "lower: " << std::get<0>(boundTuple) << " elev: " << std::get<1>(boundTuple) << " upper: " << std::get<2>(boundTuple) << std::endl;
//                        /// DEBUG!
//                        std::cout << "RIGHTSIDE \n";
//                        double lower = std::get<0>(boundTuple);
//                        double elev = std::get<1>(boundTuple);
//                        double upper = std::get<2>(boundTuple);
//                        weightedDifference = gaussianWeightedDifferenceIncrement(lower, elev, upper, diff);
//                        std::cout << "Weighted Height Difference: " << weightedDifference << "\n";
//                        weightedDifferenceVector_.push_back(heightDifferenceFromComparison_ + weightedDifference); // TODO: Check Viability
//                        if(weightedDifferenceVector_.size() >= 6) weightedDifferenceVector_.erase(weightedDifferenceVector_.begin());
//                    }

//                    // Preparing integral part for PID
//                    if(fabs(weightedDifference) < 10.0){
//                        totalHeightDifference_ += weightedDifference;
//                        heightDifferenceComponentCounter_ += 1;
//                    }


//                    // Filter for drift estimation.
//                    if(std::isnan(estimatedDrift_) || std::isnan(estimatedDriftVariance_)){
//                        estimatedDrift_ = 0;
//                        estimatedDriftVariance_ = 0;
//                    }
//                    estimatedDrift_, estimatedDriftVariance_ = filteredDriftEstimation(weightedDifference, estimatedDrift_, estimatedDriftVariance_);
//                }
//                else std::cout << "posLEft is outside of range!" << std::endl;
//            }
//        }
//    }

    //  if(rawMap_.getSize()(0) > 1){
//          // Actual Difference quantification:
//          float heightDiff, heightDiffPID;
//          if(heightDifferenceComponentCounter_ > 0.0){
//              heightDiff = totalHeightDifference_ / double(heightDifferenceComponentCounter_);
//              heightDiffPID = differenceCalculationUsingPID();
//          }
//          else heightDiffPID = oldDiffPID_;
//          std::cout << "This is PIDed height difference: " << heightDiffPID << std::endl;


//          // Old diff assignment for differential parts of the controllers.
//         // oldDiff_ = weightedDifference;
//          oldDiffPID_ = heightDiffPID;
//          // Assign class Variable.

//          //heightDifferenceFromComparison_ = heightDiff;


//          // TODO: Check if zero or nan, e.g. foot tip outside of defined elevation map region and leave at old value
//          //! TEST:
//          //if(weightedDifference >= 0.00001)

//          // Only update the Difference error id both foot tips are inside the elevation map area.
//          if(isnan()) std::cout << "FOOT TIP OUTSIDE KNOWN ELEVATION MAP ISNAN \n";
//          else heightDifferenceFromComparison_ = heightDiffPID; // Cumulative adding of height..
//          //! END TEST


//          std::cout << "Height Diff after calculation: " << heightDiff << "and Height diff PID: " << heightDiffPID << std::endl;

          // TESTING
          //rawMap_.add("elevation_fast_addition"); // TODO: think if this only adds a layer or actually adds height to "elevation"
          //grid_map::GridMap map;

          //frameCorrection();

          // IF NOT ADDING MAPS, but translating frames.
//          if(false) rawMap_["elevation_corrected"] = rawMap_["elevation"]; // HACKED!!!
//          else{
//              rawMap_["elevation_corrected"].setConstant(heightDifferenceFromComparison_); // HACKED FOR TESTING!! -> TODO: create nice Kalman Filter!!
//              //for(unsigned int n = 0; n < rawMap_.getLayers().size(); ++n){
//              //    std::cout << "LAYER: " << n << " :" << rawMap_.getLayers()[n] << std::endl;
//              //}
//              rawMap_["elevation_corrected"] = rawMap_["elevation"] + rawMap_["elevation_corrected"];
//          }

//          grid_map_msgs::GridMap mapMessage;
//          GridMapRosConverter::toMessage(rawMap_, mapMessage);



          // Listen to tf to transform the grid map message
//          tf::StampedTransform trans;
//          try{
//                baseOdomTransformListener_.lookupTransform("/odom", "odom_z_corrected",
//                                         ros::Time(0), trans);
//              }
//              catch (tf::TransformException ex){
//                ROS_ERROR("%s",ex.what());
//              }
//          std::cout << "TRANSFORM LISTENED: z: " << trans.getOrigin()[2] << std::endl;
//          std::cout << "VS. Height Difference: " << heightDifferenceFromComparison_ << std::endl;




          // Concept: publish elevation map in map_corrected
          //if(transformInCorrectedFrame_) mapMessage.info.header.frame_id = "odom_z_corrected";


      //    elevationMapFastPublisher_.publish(mapMessage);
          //std::cout << "CORRECTED!!" << std::endl;
          //heightDifferenceComponentCounter_ = 0;
          //totalHeightDifference_ = 0.0;

          /// SOME TESTS:
          ///
    //      tf::StampedTransform trans;
    //      try{
    //            baseOdomTransformListener_.lookupTransform("/odom", "odom_z_corrected",
    //                                     ros::Time(0), trans);
    //          }
    //          catch (tf::TransformException ex){
    //            ROS_ERROR("%s",ex.what());
    //          }
    //      std::cout << "TRANSFORM LISTENED: z: " << trans.getOrigin()[2] << std::endl;
    //      std::cout << "VS. Height Difference: " << heightDifferenceFromComparison_ << std::endl;

         // std::cout << "TRANSFORM LISTENED: frameid: " << trans.frame_id_ << std::endl;
         // std::cout << "TRANSFORM LISTENED: x: " << odomMapTransform.getOrigin()[0] << std::endl;

          ///
          /// END TESTS
   // }

    return true;
}

bool ElevationMap::initializeFootTipMarkers()
{

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
    footContactMarkerList_.color.r = 0.0;
    footContactMarkerList_.color.g = 1.0;
    footContactMarkerList_.color.b = 0.7;

    return true;
}

double ElevationMap::filteredDriftEstimation(double diff, float estDrift, float PEstDrift)
{
    float predDrift, PPredDrift, measDrift, PMeasDrift;
  //  measurement = newheightdiff - oldheightdiff
    float diffMeasurement = diff - oldDiff_;
    std::cout << "diff measurement: " << diffMeasurement << std::endl;
    std::cout << "diff old: " << oldDiff_ << std::endl;
    std::cout << "est Drift: " << estDrift << std::endl;
    float R = 0.5;
    float Q = 0.01;
//    // Prediction Step.
    predDrift = estDrift;
    PPredDrift = PEstDrift + Q;

//    // Measurement Step.
    float K = PPredDrift / (PPredDrift+R);
    std::cout << "K: " << K << std::endl;
    measDrift = predDrift + K * diffMeasurement;
    PMeasDrift = PPredDrift - K * PPredDrift;


    std::cout << "mean measDrift: " << measDrift << " per stance" << std::endl;
    // Attention: if nans are there, then drift is set to zero!
    return measDrift, PMeasDrift;
}

std::tuple<double, double, double> ElevationMap::getFusedCellBounds(const Eigen::Vector2d& position, const Eigen::Array2d& length)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipComparison(footTipStanceComparisonMutex_);
    float upperFused, lowerFused, elevationFused;
    bool doFuseEachStep = true;
    if(rawMap_.isInside(position) && heightDifferenceFromComparison_ == heightDifferenceFromComparison_ && doFuseEachStep){
        fuseArea(position, length);
        elevationFused = fusedMap_.atPosition("elevation", position);
        lowerFused = fusedMap_.atPosition("lower_bound", position);
        upperFused = fusedMap_.atPosition("upper_bound", position);
    }
    return std::make_tuple(lowerFused, elevationFused, upperFused);
}

bool ElevationMap::frameCorrection()
{
    // Transform Broadcaster for the /odom_z_corrected frame.
    tf::Transform odomMapTransform;

    odomMapTransform.setIdentity();
    odomMapTransform.getOrigin()[2] += heightDifferenceFromComparison_;

    ros::Time stamp = ros::Time().fromNSec(fusedMap_.getTimestamp());

    //std::cout << "TIMESTAMP PUBLISHED THE odom_drift_adjusted TRANSFORM!!: " << stamp << std::endl;

    mapCorrectedOdomTransformBroadcaster_.sendTransform(tf::StampedTransform(odomMapTransform,
                                          ros::Time().fromNSec(rawMap_.getTimestamp()), "odom", "odom_drift_adjusted"));

    return true;
}

float ElevationMap::differenceCalculationUsingPID()
{
    // TODO: Tune these, they are only guessed so far.
    float kp = 0.3;
    float ki = 0.7;
    float kd = 0.1;

    // Nan prevention.
    if(weightedDifferenceVector_.size() > 1 && isnan(weightedDifferenceVector_[1])) return 0.0;
    else if(weightedDifferenceVector_.size() > 1){
    // Calculate new total diff here
    double totalDiff = 0.0;
    for (auto& n : weightedDifferenceVector_)
        totalDiff += n;
    double meanDiff = totalDiff / float(weightedDifferenceVector_.size());
    return kp * weightedDifferenceVector_[0] + ki * meanDiff + kd * (weightedDifferenceVector_[0] - weightedDifferenceVector_[1]);
    }
}

float ElevationMap::gaussianWeightedDifferenceIncrement(double lowerBound, double elevation, double upperBound, double diff)
{
    diff -= heightDifferenceFromComparison_;
    // Error difference weighted as (1 - gaussian), s.t. intervall from elevation map to bound contains two standard deviations
    double weight = 0.0;
    float standardDeviationFactor = 0.5;  //! THIS MAY BE A LEARNING FUNCTION IN FUTURE!!
    if(diff < 0.0){
        weight = normalDistribution(diff, 0.0, fabs(elevation-lowerBound)*standardDeviationFactor);
    }
    else{
        weight = normalDistribution(diff, 0.0, fabs(elevation-upperBound)*standardDeviationFactor);
    }

    // Constrain to be maximally 1.
    if(weight > 1.0) weight = 1.0; // For security, basically not necessary (as the weighting term is left away in the normalDistribution function)

    std::cout << "WEIGHT: " << (1-weight) << " diff: " << diff << " elevation: " << elevation <<
                 " lower: " << lowerBound << " upper: " << upperBound << std::endl;
    return (1.0 - weight) * diff;
}

bool ElevationMap::differenceCalculationUsingKalmanFilter()
{


}

float ElevationMap::normalDistribution(float arg, float mean, float stdDev)
{
    double e = exp(-pow((arg-mean),2)/(2.0*pow(stdDev,2)));
    //Leaving away the weighting, as the maximum value should be one, which is sensible for weighting.
    return e; // (stdDev * sqrt(2*M_PI));
}

} /* namespace */
