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
  clear();

  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
      nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
  // TODO if (enableVisibilityCleanup_) when parameter cleanup is ready.
  visbilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

  // Foot tip position Subscriber for Foot tip - Elevation comparison
  footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &ElevationMap::footTipStanceCallback, this);

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

    //Debug
    //std::cout << "pointVariance: " << pointVariance << std::endl;
    // End Debug
    //const float& spatialpointvariance = spatialVariances(i);
      //   // Debug
      //
    //std::cout << "spatialVariances: " << spatialpointvariance << std::endl;
    // End Debug

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


  //std::cout << "Map Layers: " << rawMap_.getLayers()[0] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[1] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[2] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[3] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[4] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[5] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[6] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[7] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[8] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[9] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers()[10] << std::endl;
  //std::cout << "Map Layers: " << rawMap_.getLayers().size() << std::endl;

  // New *********************************************************************************************************************************************
  // Do Feet comparison stuff in here..?
  // TODO: Define position..
  std::cout << "Center: " << rawMap_.getPosition()(0) << " " << rawMap_.getPosition()(1) << std::endl;
  bool slowComparison = false;
  if(slowComparison){
      double totalDiff = 0;
      if(LFTipState_ == 1){
          std::cout << "Left Touching the ground" << std::endl;
          Position posLeft(LFTipPostiion_(0), LFTipPostiion_(1));
          float heightLeft = rawMap_.atPosition("elevation", posLeft);
          float varLeft = rawMap_.atPosition("variance", posLeft);

          std::cout << "Hight Left lower bound: " << heightLeft << std::endl;
          std::cout << "Hight Foot tip: " << LFTipPostiion_(2) << std::endl;
          std::cout << "Var Left: " << varLeft << std::endl;
          std::cout << "Diff: " << heightLeft - LFTipPostiion_(2) << std::endl;
          double diff = heightLeft - LFTipPostiion_(2);
          if(abs(diff) < 1.0) totalDiff += diff/2.5;
          else std::cout << "WARNING, big error" << std::endl;
      }
      else std::cout << "LEFT LIFTED!!!" << std::endl;

      if(RFTipState_ == 1){
          std::cout << "Right Touching the ground" << std::endl;
          Position posRight(RFTipPostiion_(0), RFTipPostiion_(1));
          float heightRight = rawMap_.atPosition("elevation", posRight);
          float varRight = rawMap_.atPosition("variance", posRight);

          std::cout << "Hight Right lower bound: " << heightRight << std::endl;
          std::cout << "Hight Foot tip: " << RFTipPostiion_(2) << std::endl;
          std::cout << "Var Right: " << varRight << std::endl;
          std::cout << "Diff: " << heightRight - RFTipPostiion_(2) << std::endl;
          double diff = heightRight - RFTipPostiion_(2);
          if(abs(diff) < 1.0) totalDiff += diff/2.5;
          else std::cout << "WARNING, big error" << std::endl;
      }
      else std::cout << "RIGHT LIFTED!!!" << std::endl;
      // End New **********************************************************************************************************************************************

      //rawMap_.add("elevation", -totalDiff); // HACKED!!
      //totalHeightDifference_ = 0.0;
      //std::cout << "HIGH GRASS: Shifted the Elevation Map" << std::endl;



  }

  // New: (From fast diff calculation)

//  float heightDiff;
//  if(heightDifferenceComponentCounter_ > 0.0) heightDiff = totalHeightDifference_ / double(heightDifferenceComponentCounter_);
//  else heightDiff = 0.0;
//  std::cout << "heightDiff: " << heightDiff << std::endl;
  std::cout << "Updated ##########################" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "################################################" << std::endl;
  heightDifferenceComponentCounter_ = 0;
  totalHeightDifference_ = 0.0;
  // End New

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

  // TODO: get foot tips here, and understand how variance ellipsoids are generated here..

  // Nothing to do.
  if ((size == 0).any()) return false;

  // Initializations.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);


  // TEST
  const auto& sizemap = rawMap_.getSize();

  std::cout << "Size x: " << size(0) << "Size y: " << size(1) << std::endl;
  // END TEST


  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  auto rawMapCopy = rawMap_;
  scopedLockForRawData.unlock();

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

  boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);
  //boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  const auto& sizerawmap = rawMap_.getSize();
  //std::cout << "Size of Map x: " << sizerawmap(0) << " foot tip" << std::endl;

//  std::cout << rawMap_.getLayers()[0] << std::endl;
//  std::cout << rawMap_.getLayers()[1] << std::endl;
//  std::cout << rawMap_.getLayers()[2] << std::endl;
//  std::cout << rawMap_.getLayers()[3] << std::endl;
//  std::cout << rawMap_.getLayers()[4] << std::endl;
//  std::cout << rawMap_.getLayers()[5] << std::endl;
//  std::cout << rawMap_.getLayers()[6] << std::endl;
//  std::cout << rawMap_.getLayers()[7] << std::endl;
//  std::cout << rawMap_.getLayers()[8] << std::endl;
//  std::cout << rawMap_.getLayers()[9] << std::endl;

  // Set class variables.
  LFTipPostiion_(0) = quadrupedState.contacts[0].position.x;
  LFTipPostiion_(1) = quadrupedState.contacts[0].position.y;
  LFTipPostiion_(2) = quadrupedState.contacts[0].position.z;
  RFTipPostiion_(0) = quadrupedState.contacts[1].position.x;
  RFTipPostiion_(1) = quadrupedState.contacts[1].position.y;
  RFTipPostiion_(2) = quadrupedState.contacts[1].position.z;
  LFTipState_ = quadrupedState.contacts[0].state;
  RFTipState_ = quadrupedState.contacts[1].state;


  //std::cout << "RFTippos x: " << RFTipPostiion_(0) << std::endl;

  //std::cout << "LFTipsStance Size: " << LFTipStance_.size() << std::endl;



  // Detect start and end of stances.
  detectStancePhase();

  //std::cout << "Tipstate " << LFTipState_ << std::endl;
  //std::cout << "Tipstate r " << RFTipState_ << std::endl;




  //if(processStanceTriggerLeft_.size() >= 2) std::cout << "late end: " << processStanceTriggerLeft_.back() << std::endl;


  //TODO: Think of Method to get robust reaction to changing stance state!!






//  for(unsigned int i; i <= 3; ++i){
//      geometry_msgs::Point p;
//      p.x = quadrupedState.contacts[i].position.x;
//      p.y = quadrupedState.contacts[i].position.y;
//      p.z = quadrupedState.contacts[i].position.z;

//      if(quadrupedState.contacts[i].state == 1){
//          footContactMarkerList_.points.push_back(p);
//      }

//      // Clear some foot tip markers after a certain time (and move them to the beginning of the list, as no pop_front exists)
//      int size =footContactMarkerList_.points.size();
//      if(size > 1200){
//          for(unsigned int j = 0; j<400; ++j){
//              footContactMarkerList_.points[j] = footContactMarkerList_.points[size - 400 + j];
//          }
//          for (unsigned int n = 0; n < (size-400); ++n){
//              footContactMarkerList_.points.pop_back();
//          }
//      }
//  }




  footTipElevationMapComparison();


}

bool ElevationMap::detectStancePhase(){

    //! TEST about the two threads
    //std::thread::id this_id = std::this_thread::get_id();
    //std::cout << "This is the thread: " << this_id << std::endl;
    //! END TEST

    // Collect State Data for stance phase detection
    processStanceTriggerLeft_.push_back(LFTipState_);
    processStanceTriggerRight_.push_back(RFTipState_);

    if(processStanceTriggerLeft_.size() > 1000){
        processStanceTriggerLeft_.erase(processStanceTriggerLeft_.begin());
    }
    if(processStanceTriggerRight_.size() > 1000){
        processStanceTriggerRight_.erase(processStanceTriggerRight_.begin());
    }



    // Collect the foot tip position data (if foot tip in contact).
    if(LFTipState_ && isInStanceLeft_){
        LFTipStance_.push_back(LFTipPostiion_);
        //std::cout << "LFTipStance.x: " << LFTipStance_.back()(0) << "y: " << LFTipStance_.back()(1) << "z: " << LFTipStance_.back()(2) << std::endl;
    }
    if(RFTipState_ && isInStanceRight_){
        RFTipStance_.push_back(RFTipPostiion_);
        std::cout << "RTipPosition x: " << RFTipPostiion_(0) << std::endl;
    }



    // TODO: Make nicer and more compact AND CORRECT< SWAP THEM!!



    // Recognition of the end of a stance phase of the front legs.
    if(processStanceTriggerLeft_.size() >= 20 && processStanceTriggerLeft_.end()[-1]+processStanceTriggerLeft_.end()[-2]+
            processStanceTriggerLeft_.end()[-3]+processStanceTriggerLeft_.end()[-4]+processStanceTriggerLeft_.end()[-5]+
            processStanceTriggerLeft_.end()[-6]+processStanceTriggerLeft_.end()[-7]+processStanceTriggerLeft_.end()[-8]+
            processStanceTriggerLeft_.end()[-9]+processStanceTriggerLeft_.end()[-10] >= 8 &&
            processStanceTriggerLeft_.end()[-11]+processStanceTriggerLeft_.end()[-12]+ processStanceTriggerLeft_.end()[-13]+
            processStanceTriggerLeft_.end()[-14]+processStanceTriggerLeft_.end()[-15]+processStanceTriggerLeft_.end()[-16] +
            processStanceTriggerLeft_.end()[-17]+processStanceTriggerLeft_.end()[-18]+processStanceTriggerLeft_.end()[-19] <= 2 &&
            !isInStanceLeft_){
        std::cout << "Start of LEFT stance" << std::endl;
        isInStanceLeft_ = 1;
    }



    if(processStanceTriggerRight_.size() >= 20 && processStanceTriggerRight_.end()[-1]+processStanceTriggerRight_.end()[-2]+
            processStanceTriggerRight_.end()[-3]+processStanceTriggerRight_.end()[-4]+processStanceTriggerRight_.end()[-5] +
            processStanceTriggerRight_.end()[-6]+processStanceTriggerRight_.end()[-7]+processStanceTriggerRight_.end()[-8]+
            processStanceTriggerRight_.end()[-9]+processStanceTriggerRight_.end()[-10] >= 8 &&
            processStanceTriggerRight_.end()[-11]+processStanceTriggerRight_.end()[-12]+processStanceTriggerRight_.end()[-13]+
            processStanceTriggerRight_.end()[-14]+processStanceTriggerRight_.end()[-15]+processStanceTriggerRight_.end()[-16]+
            processStanceTriggerRight_.end()[-17]+processStanceTriggerRight_.end()[-18]+processStanceTriggerRight_.end()[-19] <= 2 &&
            !isInStanceRight_){
        std::cout << "Start of RIGHT stance" << std::endl;
        isInStanceRight_ = 1;
    }

    // Recognition of the start of a stance phase of the front legs.
    if(processStanceTriggerLeft_.size() >= 20 &&
            processStanceTriggerLeft_.end()[-1] + processStanceTriggerLeft_.end()[-2] + processStanceTriggerLeft_.end()[-3]+
            processStanceTriggerLeft_.end()[-4]+processStanceTriggerLeft_.end()[-5] + processStanceTriggerLeft_.end()[-6]+
            processStanceTriggerLeft_.end()[-7]+processStanceTriggerLeft_.end()[-8]+
            processStanceTriggerLeft_.end()[-9]+processStanceTriggerLeft_.end()[-10] <= 2 &&
            processStanceTriggerLeft_.end()[-11]+processStanceTriggerLeft_.end()[-12]+ processStanceTriggerLeft_.end()[-13]+
            processStanceTriggerLeft_.end()[-14]+processStanceTriggerLeft_.end()[-15]+processStanceTriggerLeft_.end()[-16] +
            processStanceTriggerLeft_.end()[-17]+processStanceTriggerLeft_.end()[-18]+processStanceTriggerLeft_.end()[-19] >=8 &&
            isInStanceLeft_){
        if(!processStance("Left")) return false;
        isInStanceLeft_ = 0;
    }

    // Recognition of the start of a stance phase of the front legs.
    if(processStanceTriggerRight_.size() >= 20 &&
            processStanceTriggerRight_.end()[-1] + processStanceTriggerRight_.end()[-2] + processStanceTriggerRight_.end()[-3]+
            processStanceTriggerRight_.end()[-4]+processStanceTriggerRight_.end()[-5] + processStanceTriggerRight_.end()[-6]+
            processStanceTriggerRight_.end()[-7]+processStanceTriggerRight_.end()[-8]+
            processStanceTriggerRight_.end()[-9]+processStanceTriggerRight_.end()[-10] <= 2 &&
            processStanceTriggerRight_.end()[-11]+processStanceTriggerRight_.end()[-12]+processStanceTriggerRight_.end()[-13]+
            processStanceTriggerRight_.end()[-14]+processStanceTriggerRight_.end()[-15]+processStanceTriggerRight_.end()[-16]+
            processStanceTriggerRight_.end()[-17]+processStanceTriggerRight_.end()[-18]+processStanceTriggerRight_.end()[-19] >=8 &&
            isInStanceRight_){
        if(!processStance("Right")) return false;
        isInStanceRight_ = 0;
    }


    //std::cout << "LFTipState: " << LFTipState_ << std::endl;
    //std::cout << "Is in stance Left: " << isInStanceLeft_ << std::endl;
    //std::cout << "RFTipState: " << RFTipState_ << std::endl;
    //std::cout << "Is in stance Right: " << isInStanceRight_ << std::endl;


    //std::cout << "LFTipsStance Size: " << LFTipStance_.size() << std::endl;


    //std::cout << "Is it processing?: " << isProcessingRight_ << std::endl;
    //std::cout << "processStanceTrigger size: " << processStanceTriggerRight_.size() << std::endl;
    //std::cout << "Z coordinate: " << quadrupedState.contacts[1].position.z << std::endl;

    // TODO: check threading!! Why calls them always twice??


    return true;
}

bool ElevationMap::processStance(std::string tip)
{
    std::cout << "Processing: " << tip <<std::endl;

    // Process here..

    //std::cout << "LFTipsStance Size: " << LFTipStance_.size() << std::endl;

    double xMeanLeft, yMeanLeft, zMeanLeft, xMeanRight, yMeanRight, zMeanRight;
    double xTotalLeft, yTotalLeft, zTotalLeft, xTotalRight, yTotalRight, zTotalRight;

    if(tip == "Left"){
        for (auto& n : LFTipStance_){
            // Consider only those with state = 1
            xTotalLeft += n(0);
            yTotalLeft += n(1);
            zTotalLeft += n(2);
        }
        xMeanLeft = xTotalLeft / float(LFTipStance_.size());
        yMeanLeft = yTotalLeft / float(LFTipStance_.size());
        zMeanLeft = zTotalLeft / float(LFTipStance_.size());

        std::cout << "x Mean Left: " << xMeanLeft << std::endl;
        std::cout << "y Mean Left: " << yMeanLeft << std::endl;
        std::cout << "z Mean Left: " << zMeanLeft << std::endl;

        std::cout << "LSIZE: " << LFTipStance_.size() << std::endl;
        LFTipStance_.clear();
        processStanceTriggerLeft_.clear();
        std::cout << "LSIZE: " << LFTipStance_.size() << std::endl;
    }

    if(tip == "Right"){
        for (auto& n : RFTipStance_){
            // Consider only those with state = 1
            xTotalRight += n(0);
            yTotalRight += n(1);
            zTotalRight += n(2);
        }
        xMeanRight = xTotalRight / float(RFTipStance_.size());
        yMeanRight = yTotalRight / float(RFTipStance_.size());
        zMeanRight = zTotalRight / float(RFTipStance_.size());

        std::cout << "x Mean Right: " << xMeanRight << std::endl;
        std::cout << "y Mean Right: " << yMeanRight << std::endl;
        std::cout << "z Mean Right: " << zMeanRight << std::endl;

        std::cout << "RSIZE: " << RFTipStance_.size() << std::endl;
        RFTipStance_.clear();
        processStanceTriggerRight_.clear();
        std::cout << "RSIZE: " << RFTipStance_.size() << std::endl;
    }


    //std::cout << "LFTipsStance Size: " << LFTipStance_.size() << std::endl;

    return true;
}

bool ElevationMap::footTipElevationMapComparison(){


    //footContactPublisher_.publish(footContactMarkerList_);
    //const auto& size = rawMap_.getSize();
    //std::cout << "Size x: " << size(0) << "Size y: " << size(1) << std::endl;

    bool print_horizontal_variances = false;

    // Test:
    bool fastComparison = true;
    if(fastComparison){
        if(LFTipState_ == 1){
            //std::cout << "Left Touching the ground" << std::endl;
            //std::cout << "Left pos x: " << LFTipPostiion_(0) << "Right Pos y: " << LFTipPostiion_(1) << std::endl;
            Position posLeft(LFTipPostiion_(0), LFTipPostiion_(1));
            //std::cout << "before" << std::endl;
            // For some reason the rawMap is sometimes empty..
            if(rawMap_.getSize()(0) > 1){
                //std::cout << "Left pos x: " << LFTipPostiion_(0) << "Right Pos y: " << LFTipPostiion_(1) << std::endl;

                //std::cout << "Positions: " << posLeft(0) << " " << posLeft(1) << std::endl;
                //std::cout << "Center: " << rawMap_.getPosition()(0) << " " << rawMap_.getPosition()(1) << std::endl;
                //std::cout << "Size: " << rawMap_.getLength() << std::endl;
                float heightLeft = rawMap_.atPosition("elevation", posLeft);
                float varLeft = rawMap_.atPosition("variance", posLeft);
                //std::cout << "after" << std::endl;

                //std::cout << "Height Left lower bound: " << heightLeft << std::endl;
                //std::cout << "Height Foot tip: " << LFTipPostiion_(2) << std::endl;
                //std::cout << "Var Left: " << varLeft << std::endl;
                //std::cout << "Diff: " << heightLeft - LFTipPostiion_(2) << std::endl;

                // TEST:
                if(LFTipPostiion_(2) < heightLeft - varLeft || LFTipPostiion_(2) > heightLeft + varLeft){
                    //std::cout << "Right height is critical!!!!! *************************************************" << std::endl;
                }

                // Output variance values:
                if(print_horizontal_variances){
                    std::cout << "Left: horizontal_variance_x: " << rawMap_.atPosition("horizontal_variance_x",posLeft) << std::endl;
                    std::cout << "Left: horizontal_variance_y: " << rawMap_.atPosition("horizontal_variance_y",posLeft) << std::endl;
                    std::cout << "Left: horizontal_variance_xy: " << rawMap_.atPosition("horizontal_variance_xy",posLeft) << std::endl;
                }


                double diff = heightLeft - LFTipPostiion_(2);
                if(abs(diff) < 1.0) totalHeightDifference_ += diff;
                heightDifferenceComponentCounter_ += 1;
                //else std::cout << "WARNING, big error" << std::endl;
            }
        }
        //else std::cout << "LEFT LIFTED!!!" << std::endl;

        if(RFTipState_ == 1){
            //std::cout << "Right Touching the ground" << std::endl;
            Position posRight(RFTipPostiion_(0), RFTipPostiion_(1));
            // For some reason the rawMap is sometimes empty..
            if(rawMap_.getSize()(0) > 1){
                float heightRight = rawMap_.atPosition("elevation", posRight);

                float varRight = rawMap_.atPosition("variance", posRight);
                float horVarRight = rawMap_.atPosition("horizontal_variance_x", posRight);

                //std::cout << "Height Right lower bound: " << heightRight << std::endl;
                //std::cout << "Height Foot tip: " << RFTipPostiion_(2) << std::endl;
                //std::cout << "Var Right: " << varRight << std::endl;
                //std::cout << "Diff: " << heightRight - RFTipPostiion_(2) << std::endl;

                //std::cout << "Variance varying? " << horVarRight << std::endl;
                // TEST:
                if(RFTipPostiion_(2) < heightRight - varRight || RFTipPostiion_(2) > heightRight + varRight){
                    //std::cout << "Right height is critical!!!!! *************************************************" << std::endl;
                }

                if(print_horizontal_variances){
                    std::cout << "Right: horizontal_variance_x: " << rawMap_.atPosition("horizontal_variance_x",posRight) << std::endl;
                    std::cout << "Right: horizontal_variance_y: " << rawMap_.atPosition("horizontal_variance_y",posRight) << std::endl;
                    std::cout << "Right: horizontal_variance_xy: " << rawMap_.atPosition("horizontal_variance_xy",posRight) << std::endl;
                }

                double diff = heightRight - RFTipPostiion_(2);
                if(abs(diff) < 10.0) totalHeightDifference_ += diff;
                heightDifferenceComponentCounter_ += 1;
                //else std::cout << "WARNING, big error" << std::endl;
            }
        }
        //else std::cout << "RIGHT LIFTED!!!" << std::endl;
        // End Test!!

    }

    //std::cout << "Frame transform z odom map " << quadrupedState.frame_transforms.at(0).transform.translation.z << std::endl;
    //std::cout << "Frame transform z odom map_ga " << quadrupedState.frame_transforms.at(1).transform.translation.z << std::endl;

    //! To add: consider horizontal variance for diff calculation
    //! How is frequency of motion update.. Where stored?

    if(rawMap_.getSize()(0) > 1){
      // Actual Difference quantification:
      float heightDiff;
      if(heightDifferenceComponentCounter_ > 0.0) heightDiff = totalHeightDifference_ / double(heightDifferenceComponentCounter_);
      else heightDiff = 0.0;
      //std::cout << "heightDiff: " << heightDiff << "totDiff: " << totalHeightDifference_ << std::endl;
      if(heightDifferenceComponentCounter_ > 30){
          totalHeightDifference_ = 0.0;
          heightDifferenceComponentCounter_ = 0;
      }
      //std::cout << "heightDiff: " << heightDiff << std::endl;

      //heightDifferenceComponentCounter_ = 0;
      //totalHeightDifference_ = 0.0;
    }







    return true;
}

} /* namespace */
