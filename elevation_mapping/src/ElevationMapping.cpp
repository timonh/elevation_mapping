/*
 * ElevationMapping.cpp
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
using namespace kindr;
using namespace kindr_ros;

namespace elevation_mapping {

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),

      stanceProcessor_(nodeHandle),
      supportSurfaceEstimation_(nodeHandle),

      isContinouslyFusing_(false),
      ignoreRobotMotionUpdates_(false)
{
  ROS_INFO("Elevation mapping node started.");
  readParameters();
  // TESTS
  ROS_WARN_STREAM("Elevation mapping topic name: " << pointCloudTopic_);
  //
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
  if (!robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  } else {
    ignoreRobotMotionUpdates_ = true;
  }

  mapUpdateTimer_ = nodeHandle_.createTimer(maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);

  // Multi-threading for fusion.
  AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = AdvertiseServiceOptions::create<std_srvs::Empty>(
      "trigger_fusion", boost::bind(&ElevationMapping::fuseEntireMap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForTriggerFusion);

  AdvertiseServiceOptions advertiseServiceOptionsForGetSubmap = AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_submap", boost::bind(&ElevationMapping::getSubmap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  submapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetSubmap);

  // New for getting support surface map.
  AdvertiseServiceOptions advertiseServiceOptionsForGetSupportSurfaceSubmap = AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_support_surface_submap", boost::bind(&ElevationMapping::getSupportSurfaceSubmap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  supportSurfaceSubmapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetSupportSurfaceSubmap);

  if (!fusedMapPublishTimerDuration_.isZero()) {
    TimerOptions timerOptions = TimerOptions(
        fusedMapPublishTimerDuration_,
        boost::bind(&ElevationMapping::publishFusedMapCallback, this, _1), &fusionServiceQueue_,
        false, false);
    fusedMapPublishTimer_ = nodeHandle_.createTimer(timerOptions);
  }

  // Multi-threading for visibility cleanup.
  if (map_.enableVisibilityCleanup_ && !visibilityCleanupTimerDuration_.isZero()){
    TimerOptions timerOptions = TimerOptions(
        visibilityCleanupTimerDuration_,
        boost::bind(&ElevationMapping::visibilityCleanupCallback, this, _1), &visibilityCleanupQueue_,
        false, false);
    visibilityCleanupTimer_ = nodeHandle_.createTimer(timerOptions);
  }

  clearMapService_ = nodeHandle_.advertiseService("clear_map", &ElevationMapping::clearMap, this);
  saveMapService_ = nodeHandle_.advertiseService("save_map", &ElevationMapping::saveMap, this);

  // Parameters. // TODO: when complete move to read parameters
  nodeHandle_.param("use_bag", useBag_, false); // SP
  nodeHandle_.param("run_foot_tip_elevation_map_enhancements", runFootTipElevationMapEnhancements_, true); // SP
  nodeHandle_.param("run_hind_leg_stance_detection", runHindLegStanceDetection_, true); // SP
  nodeHandle_.param("run_support_surface_estimation", runSupportSurfaceEstimation_, false);

  //! Introduced by Timon
  if (true) {
    if(runFootTipElevationMapEnhancements_){ // SP
      if(!useBag_) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 28, &ElevationMapping::footTipStanceCallback, this);
      else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 28, &ElevationMapping::footTipStanceCallback, this);
    } // SP
  }
  //! End of newly introduced section

  // Do adjust the queue size if needed TODO:
  processTriggerSubscriber_ = nodeHandle_.subscribe("stance_trigger", 20, &ElevationMapping::processTriggerCallback, this);

  // Publisher for data on continuity assumption negotiation.
  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);


  initialize();
}

ElevationMapping::~ElevationMapping()
{
  // New added to write data to file for learning.
  map_.writeDataFileForParameterLearning();

  fusionServiceQueue_.clear();
  fusionServiceQueue_.disable();
  nodeHandle_.shutdown();
  fusionServiceThread_.join();
}

bool ElevationMapping::readParameters()
{
  // ElevationMapping parameters.

  nodeHandle_.param("point_cloud_topic", pointCloudTopic_, string("/points"));
  nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/robot"));
  nodeHandle_.param("track_point_x", trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", trackPoint_.z(), 0.0);

  nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  ROS_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  nodeHandle_.param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);

  double fusedMapPublishingRate;
  nodeHandle_.param("fused_map_publishing_rate", fusedMapPublishingRate, 1.0);
  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_.fromSec(0.0);
    ROS_WARN("Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is called.");
  } else if (std::isinf(fusedMapPublishingRate)){
    isContinouslyFusing_ = true;
    fusedMapPublishTimerDuration_.fromSec(0.0);
  } else {
    fusedMapPublishTimerDuration_.fromSec(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate;
  nodeHandle_.param("visibility_cleanup_rate", visibilityCleanupRate, 1.0);
  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_.fromSec(0.0);
    ROS_WARN("Rate for visibility cleanup is zero and therefore disabled.");
  }
  else {
    visibilityCleanupTimerDuration_.fromSec(1.0 / visibilityCleanupRate);
    map_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }


  // ElevationMap parameters. TODO Move this to the elevation map class.
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  map_.setFrameId(frameId);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  map_.setGeometry(length, resolution, position);

  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);
  nodeHandle_.param("underlying_map_topic", map_.underlyingMapTopic_, string());
  nodeHandle_.param("enable_visibility_cleanup", map_.enableVisibilityCleanup_, true);
  nodeHandle_.param("scanning_duration", map_.scanningDuration_, 1.0);

  // SensorProcessor parameters.
  string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, string("structured_light"));
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, transformListener_));
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) return false;
  if (!robotMotionMapUpdater_.readParameters()) return false;

  return true;
}

bool ElevationMapping::initialize()
{
  ROS_INFO("Elevation mapping node initializing ... ");
  fusionServiceThread_ = boost::thread(boost::bind(&ElevationMapping::runFusionServiceThread, this));
  Duration(1.0).sleep(); // Need this to get the TF caches fill up.  // HACKEDHACKED
  resetMapUpdateTimer();
  fusedMapPublishTimer_.start();
  visibilityCleanupThread_ = boost::thread(boost::bind(&ElevationMapping::visibilityCleanupThread, this));
  visibilityCleanupTimer_.start();
  ROS_INFO("Done.");
  return true;
}

void ElevationMapping::runFusionServiceThread()
{
  static const double timeout = 0.05;

  while (nodeHandle_.ok()) {
    fusionServiceQueue_.callAvailable(ros::WallDuration(timeout));
  }
}

void ElevationMapping::visibilityCleanupThread()
{
  static const double timeout = 0.05;

  while (nodeHandle_.ok()) {
    visibilityCleanupQueue_.callAvailable(ros::WallDuration(timeout));
  }
}


void ElevationMapping::pointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{
  stopMapUpdateTimer();

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(rawPointCloud, pcl_pc);

  PointCloud<PointXYZRGB>::Ptr pointCloud(new PointCloud<PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  ROS_INFO("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));





  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!ignoreRobotMotionUpdates_) {
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty? 1st", lastPointCloudUpdateTime_.toSec());
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloud<PointXYZRGB>::Ptr pointCloudProcessed(new PointCloud<PointXYZRGB>);
  Eigen::VectorXf measurementVariances;
  // New *******************
  Eigen::VectorXf spatialVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed,
                                 measurementVariances, spatialVariances)) {
    ROS_ERROR("Point cloud could not be processed.");
    resetMapUpdateTimer();
    return;
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, spatialVariances, lastPointCloudUpdateTime_, Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    ROS_ERROR("Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }



  // Publish elevation map.
  map_.publishRawElevationMap();
  if (isContinouslyFusing_ && map_.hasFusedMapSubscribers()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();

}

void ElevationMapping::mapUpdateTimerCallback(const ros::TimerEvent&)
{
  ROS_WARN("Elevation map is updated without data from the sensor.");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();
  ros::Time time = ros::Time::now();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.publishRawElevationMap();
  if (isContinouslyFusing_ && map_.hasFusedMapSubscribers()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback(const ros::TimerEvent&)
{
  if (!map_.hasFusedMapSubscribers()) return;
  ROS_DEBUG("Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback(const ros::TimerEvent&)
{
  ROS_DEBUG("Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_.visibilityCleanup(ros::Time(lastPointCloudUpdateTime_));
}

bool ElevationMapping::fuseEntireMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
  return true;
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  if (ignoreRobotMotionUpdates_) return true;

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  // Get robot pose at requested time.
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
    return false;
  }

  HomTransformQuatD robotPose;
  convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance = Eigen::Map<
      const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = ros::Time(0);
  convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Position3D position3d;
  convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  Index index;
  GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    vector<string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;
}

// New for Locomotion controller with support surface. TODO: check if the mapfusionqueue is suitable for this cause..
bool ElevationMapping::getSupportSurfaceSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  //boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  //map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  Index index;
  GridMap subMap = map_.getSupportSurfaceGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  //scopedLock.unlock();

  if (request.layers.empty()) {
    GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    vector<string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  //ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;
}

bool ElevationMapping::clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Clearing map.");
  return map_.clear();
}

bool ElevationMapping::saveMap(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response)
{
  ROS_INFO("Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = nodeHandle_.getNamespace() + "/elevation_map";
  response.success = GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request.file_path, topic);
  response.success = GridMapRosConverter::saveToBag(map_.getRawGridMap(), request.file_path + "_raw", topic + "_raw");
  return response.success;
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

void ElevationMapping::footTipStanceCallback(const quadruped_msgs::QuadrupedState& quadrupedState) // SP
{
  //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);
  // Set class variables.

  stanceProcessor_.LFTipPosition_(0) = (double)quadrupedState.contacts[0].position.x;
  stanceProcessor_.LFTipPosition_(1) = (double)quadrupedState.contacts[0].position.y;
  stanceProcessor_.LFTipPosition_(2) = (double)quadrupedState.contacts[0].position.z;
  stanceProcessor_.RFTipPosition_(0) = (double)quadrupedState.contacts[1].position.x;
  stanceProcessor_.RFTipPosition_(1) = (double)quadrupedState.contacts[1].position.y;
  stanceProcessor_.RFTipPosition_(2) = (double)quadrupedState.contacts[1].position.z;
  stanceProcessor_.LFTipState_ = quadrupedState.contacts[0].state;
  stanceProcessor_.RFTipState_ = quadrupedState.contacts[1].state;

  if(runHindLegStanceDetection_){
      // Add hind legs for proprioceptive variance estimation.
      stanceProcessor_.LHTipPosition_(0) = (double)quadrupedState.contacts[2].position.x;
      stanceProcessor_.LHTipPosition_(1) = (double)quadrupedState.contacts[2].position.y;
      stanceProcessor_.LHTipPosition_(2) = (double)quadrupedState.contacts[2].position.z;
      stanceProcessor_.RHTipPosition_(0) = (double)quadrupedState.contacts[3].position.x;
      stanceProcessor_.RHTipPosition_(1) = (double)quadrupedState.contacts[3].position.y;
      stanceProcessor_.RHTipPosition_(2) = (double)quadrupedState.contacts[3].position.z;
      stanceProcessor_.LHTipState_ = quadrupedState.contacts[2].state;
      stanceProcessor_.RHTipState_ = quadrupedState.contacts[3].state;
  }

  // Detect start and end of stances for each of the two front foot tips.
  std::string triggeredStance = stanceProcessor_.detectStancePhase();

  //std::cout << " Looping: tip: " << triggeredStance << std::endl;

  if (triggeredStance != "none") {
    //  std::cout << " BEFORE PROCESSING!!! " << triggeredStance << std::endl;
     // processTip(triggeredStance);
    //  std::cout << " AFTER!!! => PROCESSING!!! " << triggeredStance << std::endl;
  }

  //if (triggeredStance != "none") processTip(triggeredStance);
  //std::cout << "De string isch:"  << triggeredStance << std::endl;
  // Broadcast frame transform for drift adjustment.

  frameCorrection();
  //bool trigger = getFootTipTrigger();
  //std::string tip = getTriggeredTip();
  //if (trigger) {
  //    std::cout << "TIPTIPTIP:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: " << tip << std::endl;
  //    std::cout << " \n ENDEND " << std::endl;
  //    processTip(tip);
  //}
}

void ElevationMapping::processTriggerCallback(const geometry_msgs::Twist triggerTwist) {

    // TODO: this with custom message : string and eigenVector.

    //if (tip != "lefthind" && tip != "righthind") {
    std::string tip;
    if (triggerTwist.angular.x < 1.5) tip = "left";
    else if (triggerTwist.angular.x > 1.5 && triggerTwist.angular.x < 2.5) tip = "right";
    else if (triggerTwist.angular.x > 2.5 && triggerTwist.angular.x < 3.5) tip = "lefthind";
    else if (triggerTwist.angular.x > 3.5) tip = "righthind";



    Eigen::Vector3f meanStance;
    meanStance(0) = triggerTwist.linear.x;  // Check timing!!!! TODO TODO
    meanStance(1) = triggerTwist.linear.y;
    meanStance(2) = triggerTwist.linear.z;

//    if (tip == "left") {
//        stanceProcessor_.robustStanceTriggerLF_ = false;
//    }
//    else if (tip == "right") {
//        stanceProcessor_.robustStanceTriggerRF_ = false;
//    }
//    else if (tip == "lefthind") {
//        stanceProcessor_.robustStanceTriggerLH_ = false;
//    }
//    else if (tip == "righthind") {
//        stanceProcessor_.robustStanceTriggerRH_ = false;
//    }

    std::cout << "TIP PROCESSED: -> " << tip << " X value of meanstance " << meanStance(0) << std::endl;

    grid_map::Position tipPosition(meanStance(0), meanStance(1));
    // Do function get ellipsis axes for clarity..
    Eigen::Array2d ellipseAxes;
    ellipseAxes[0] = ellipseAxes[1] = std::max(6 * sqrt(map_.rawMap_.atPosition("horizontal_variance_x",tipPosition)),
                              6 * sqrt(map_.rawMap_.atPosition("horizontal_variance_y",tipPosition)));

    map_.fuseArea(tipPosition, ellipseAxes);
    // TODO: elevation map bound fusion here. and pass fused map as reference to the comparison function..
    stanceProcessor_.driftRefinement_.footTipElevationMapComparison(tip, meanStance, map_.getRawGridMap(), map_.getFusedGridMap(), map_.supportMapGP_);
    //}
    stanceProcessor_.footTipTrigger_ = false;

    stanceProcessor_.driftRefinement_.publishAveragedFootTipPositionMarkers(map_.getRawGridMap(), meanStance, tip);

    // Shift mean stance by the estimated total drift.

    // Always apply frame correction before support Surface Estimation.
    frameCorrection();

    // Support Surface Estimation.
    if (runSupportSurfaceEstimation_){
        supportSurfaceEstimation_.updateSupportSurfaceEstimation(tip, map_.getRawGridMap(), map_.supportMapGP_, map_.getFusedGridMap(), meanStance, stanceProcessor_.driftRefinement_.heightDifferenceFromComparison_);
    }
}


bool ElevationMapping::frameCorrection()
{
    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
    //boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);

    // Transform Broadcaster for the /odom_z_corrected frame.
    tf::Transform odomMapTransform;
    odomMapTransform.setIdentity();

    if (!isnan(stanceProcessor_.driftRefinement_.heightDifferenceFromComparison_)) odomMapTransform.getOrigin()[2] += stanceProcessor_.driftRefinement_.heightDifferenceFromComparison_;
    else std::cout << stanceProcessor_.driftRefinement_.heightDifferenceFromComparison_ << " <- height diff is this kind of NAN for some reason? \n ? \n ? \n";

    //ros::Time stamp = ros::Time().fromNSec(map_.fusedMap_.getTimestamp());
    mapCorrectedOdomTransformBroadcaster_.sendTransform(tf::StampedTransform(odomMapTransform,
                                          ros::Time().fromNSec(map_.rawMap_.getTimestamp()), "odom", "odom_drift_adjusted"));

    return true;
}

bool ElevationMapping::getFootTipTrigger(){
    return stanceProcessor_.footTipTrigger_;
}

std::string ElevationMapping::getTriggeredTip(){
    return stanceProcessor_.tipTrigger_;
}


} /* namespace */
