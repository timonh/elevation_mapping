/*
* ElevationMap.hpp
*
*  Created on: Feb 5, 2014
*      Author: Péter Fankhauser
*	 Institute: ETH Zurich, Autonomous Systems Lab
*/

#pragma once

// Elevation Mapping
#include <elevation_mapping/ElevationMap.hpp>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.h> // For high grass new
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp> // New for high grass
#include <grid_map_cv/InpaintFilter.hpp> // New for high grass
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Kindr
#include <kindr/Core>

// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS
#include <ros/ros.h>

// State Estimator Message
#include "quadruped_msgs/QuadrupedState.h"
#include "quadruped_msgs/Contacts.h"

// Transform Listener and broadcaster (NEW)
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

// ROS msgs
//#include <elevation_mapping/PerformanceAssessment.h>

// File IO
#include <iostream>
#include <fstream>

namespace elevation_mapping {

/*!
* Elevation map stored as grid map handling elevation height, variance, color etc.
*/
class DriftRefinement
{
public:

 /*!
  * Constructor.
  */
 DriftRefinement(ros::NodeHandle nodeHandle);

 /*!
  * Destructor.
  */
 virtual ~DriftRefinement();

 /*!
  * Set the geometry of the elevation map. Clears all the data.
  * @param length the side lengths in x, and y-direction of the elevation map [m].
  * @param resolution the cell size in [m/cell].
  * @param position the 2d position of the elevation map in the elevation map frame [m].
  * @return true if successful.
  */
 void setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position);

 /*!
  * Add new measurements to the elevation map.
  * @param pointCloud the point cloud data.
  * @param pointCloudVariances the corresponding variances of the point cloud data.
  * @param timeStamp the time of the input point cloud.
  * @param transformationSensorToMap
  * @return true if successful.
  */
 bool add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, Eigen::VectorXf& spatialVariances,
          const ros::Time& timeStamp, const Eigen::Affine3d& transformationSensorToMap);

 /*!
  * Update the elevation map with variance update data.
  * @param varianceUpdate the variance update in vertical direction.
  * @param horizontalVarianceUpdateX the variance update in horizontal x-direction.
  * @param horizontalVarianceUpdateY the variance update in horizontal y-direction.
  * @param horizontalVarianceUpdateXY the correlated variance update in horizontal xy-direction.
  * @param time the time of the update.
  * @return true if successful.
  */
 bool update(const grid_map::Matrix& varianceUpdate,
             const grid_map::Matrix& horizontalVarianceUpdateX,
             const grid_map::Matrix& horizontalVarianceUpdateY,
             const grid_map::Matrix& horizontalVarianceUpdateXY, const ros::Time& time);

 /*!
  * Triggers the fusion of the entire elevation map.
  * @return true if successful.
  */
 bool fuseAll();

 /*!
  * Fuses the elevation map for a certain rectangular area.
  * @param position the center position of the area to fuse.
  * @param length the sides lengths of the area to fuse.
  * @return true if successful.
  */
 bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length);

 /*!
  * Clears all data of the elevation map (data and time).
  * @return true if successful.
  */
 bool clear();

 /*!
  * Removes parts of the map based on visibility criterion with ray tracing.
  * @param transformationSensorToMap
  * @param updatedTime
  */
 void visibilityCleanup(const ros::Time& updatedTime);

 /*!
  * Move the grid map w.r.t. to the grid map frame.
  * @param position the new location of the elevation map in the map frame.
  */
 void move(const Eigen::Vector2d& position);

 /*!
  * Publishes the (latest) raw elevation map.
  * @return true if successful.
  */
 bool publishRawElevationMap();

 /*!
  * Publishes the fused elevation map. Takes the latest available fused elevation
  * map, does not trigger the fusion process.
  * @return true if successful.
  */
 bool publishFusedElevationMap();

 /*!
  * Publishes the (latest) visibility cleanup map.
  * @return true if successful.
  */
 bool publishVisibilityCleanupMap();

 // New ******************************
 /*!
  * Publishes a Pointcloud that is colored according to local spatial variance values
  * @return true if successful.
  */
 bool publishSpatialVariancePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& spatialVariances);
 // End New ***********************************


 /*!
  * Gets a reference to the raw grid map.
  * @return the raw grid map.
  */
 grid_map::GridMap& getRawGridMap();

 /*!
  * Gets a reference to the fused grid map.
  * @return the fused grid map.
  */
 grid_map::GridMap& getFusedGridMap();

 /*!
  * Gets the time of last map update.
  * @return time of the last map update.
  */
 ros::Time getTimeOfLastUpdate();

 /*!
  * Gets the time of last map fusion.
  * @return time of the last map fusion.
  */
 ros::Time getTimeOfLastFusion();

 /*!
  * Get the pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
  * @return pose of the elevation map frame w.r.t. the parent frame of the robot.
  */
 const kindr::HomTransformQuatD& getPose();

 /*!
  * Gets the position of a raw data point (x, y of cell position & height of cell value) in
  * the parent frame of the robot.
  * @param index the index of the requested cell.
  * @param position the position of the data point in the parent frame of the robot.
  * @return true if successful, false if no valid data available.
  */
 bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position);

 /*!
  * Gets the fused data mutex.
  * @return reference to the fused data mutex.
  */
 boost::recursive_mutex& getFusedDataMutex();

 /*!
  * Gets the raw data mutex.
  * @return reference to the raw data mutex.
  */
 boost::recursive_mutex& getRawDataMutex();

 /*!
  * Set the frame id.
  * @param frameId the frame id.
  */
 void setFrameId(const std::string& frameId);

 /*!
  * Get the frame id.
  * @return the frameId.
  */
 const std::string& getFrameId();

 /*!
  * If the raw elevation map has subscribers.
  * @return true if number of subscribers bigger then 0.
  */
 bool hasRawMapSubscribers() const;

 /*!
  * If the fused elevation map has subscribers.
  * @return true if number of subscribers bigger then 0.
  */
 bool hasFusedMapSubscribers() const;

 /*!
  * Callback method for the updates of the underlying map.
  * Updates the internal underlying map.
  * @param underlyingMap the underlying map.
  */
 void underlyingMapCallback(const grid_map_msgs::GridMap& underlyingMap);

 /*!
  * Callback Method for the quadruped state for foot tip - Elevation Map comparison
  * @param the quadruped state
  */
 void footTipStanceCallback(const quadruped_msgs::QuadrupedState& quadrupedState);

 friend class ElevationMapping;
 friend class StanceProcessor;

private:

 /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
 bool readParameters();

 /*!
  * Fuses a region of the map.
  * @param topLeftIndex the top left index of the region.
  * @param size the size (in number of cells) of the region.
  * @return true if successful.
  */
 bool fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size);

 /*!
  * Cleans the elevation map data to stay within the specified bounds.
  * @return true if successful.
  */
 bool clean();

 /*!
  * Resets the fused map data.
  * @return true if successful.
  */
 void resetFusedData();

 /*!
  * Cumulative distribution function.
  * @param x the argument value.
  * @param mean the mean of the distribution.
  * @param standardDeviation the standardDeviation of the distribution.
  * @return the function value.
  */
 float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

 //! TODO: Description!
 bool processStance(std::string tip);

 bool deleteLastEntriesOfStances(std::string tip);

 //! TODO: Description!
 bool getAverageFootTipPositions(std::string tip);

 //! TODO: Description!
 bool publishAveragedFootTipPositionMarkers(bool hind);

 //! TODO: Description!
 bool publishFusedMapBoundMarkers(double& xTip, double& yTip,
                                  double& elevationFused, double& upperBoundFused, double& lowerBoundFused);

 //! TODO: Description:
 bool detectStancePhase();

 bool templateMatchingForStanceDetection(std::string tip, std::vector<bool> &stateVector);

 //! TODO: Description:
 bool footTipElevationMapComparison(std::string mode, Eigen::Vector3f& meanStance);

 //! TODO: Description:
 std::tuple<double, double> filteredDriftEstimation(double diffComparisonUpdate, float estDrift, float PEstDrift);

 //! TODO: Description:
 bool initializeFootTipMarkers();

 //! TODO: Description:
 std::tuple<double, double, double> getFusedCellBounds(const Eigen::Vector2d& position, const Eigen::Array2d& length);

 //! TODO: Description:
 bool frameCorrection();

 //! TODO: Description:
 bool frameCorrection(double estimatedDrift);

 //! TODO: Description:
 float differenceCalculationUsingPID();

 //! TODO: Description:
 std::tuple<double, double> differenceCalculationUsingKalmanFilter();

 //! TODO: Description:
 float gaussianWeightedDifferenceIncrement(double lowerBound, double elevation, double upperbound, double diff);

 //! TODO: Description:
 float normalDistribution(float arg, float mean, float stdDev);

 //! TODO: Description:
 double driftCalculationUsingPID(std::string tip);

 //! TODO: Description:
 bool updateFootTipBasedElevationMapLayer(int numberOfConsideredFootTips);

 /*!
  * Performance assessment for tuning, only valid if walking on flat ground and therefore wishing elevation map to be flat too.
  */
 bool performanceAssessmentMeanElevationMap();

 //! TODO: Description:
 bool writeFootTipStatisticsToFile(double& footTipVal, std::string filename);

 //! TODO: Description:
 bool proprioceptiveRoughnessEstimation(std::string tip);

 //! TODO: Description:
 bool proprioceptiveVariance(std::string tip);

 //! TODO: Description:
 bool penetrationDepthEstimation(std::string tip);

 //! TODO: Description:
 bool penetrationDepthVarianceEstimation(std::string tip, double verticalDifference);

 //! TODO: Description:
 bool writeDataFileForParameterLearning();

 //! TODO: Description:
 bool updateSupportSurfaceEstimation(std::string tip);

 //! TODO: Description:
 bool penetrationDepthContinuityPropagation();

 //! TODO: Description:
 bool terrainContinuityPropagation();

 //! TODO: Description:
 bool cellPropagation(double factor1, double factor2, grid_map::Index& startingIndex, std::string propagationMethod);

 //! TODO: Description:
 bool footTipBasedElevationMapIncorporation();

 //! TODO:Description:
 void setFootTipPlaneFitCoefficients(Eigen::Vector2f& coeffs);

 //! TODO: Description:
 Eigen::Vector2f getFootTipPlaneFitCoeffcients();

 //! TODO: Description:
 void setMeanOfAllFootTips(Eigen::Vector3f& mean);

 //! TODO: Description:
 Eigen::Vector3f getMeanOfAllFootTips();

 //! TODO: Description:
 Eigen::Vector3f getMeanStance();

 //! TODO: Description:
 Eigen::Vector3f getLatestLeftStance();

 //! TODO: Description:
 Eigen::Vector3f getLatestRightStance();

 //! TODO: Description:
 void setPenetrationDepthVariance(double verticalDifference);

 //! TODO: Description:
 double getPenetrationDepthVariance();

 //! TODO: Description:
 bool penetrationDepthContinuityProcessing(std::string tip);

 //! TODO: Description:
 bool terrainContinuityProcessing();

 //! TODO: Description:
 bool footTipEmbeddingSimple();

 //! TODO: Description:
 grid_map::Position3 getFrontLeftFootTipPosition();

 //! TODO: Description:
 grid_map::Position3 getFrontRightFootTipPosition();

 //! TODO: Description:
 double getClosestMapValueUsingSpiralIterator(grid_map::GridMap& MapReference, grid_map::Position footTip, double radius, double tipHeight);

 //! The Elevation map is an upper bound to the support surface
 bool supportSurfaceUpperBounding(grid_map::GridMap& upperBoundMap, grid_map::GridMap& supportSurfaceMap);

 //!TODO: Description:
 bool addSupportSurface(grid_map::GridMap& mapSmooth, grid_map::Position leftTipHor, grid_map::Position rightTipHor, std::string tip);

 //!TODO: Description:
 bool gaussianProcessModeling();

 //!TODO: Description:
 bool setFootprint(const geometry_msgs::Transform& footprint);

 //!TODO: Description:
 geometry_msgs::Transform getFootprint();

 //! TODO: Description:
 bool gaussianProcessSmoothing(std::string tip);

 //! TODO: Description:
 bool setSmoothingTiles(double tileResolution, double tileSize, double sideLengthAddingPatch, std::string tip);

 //! TODO: Description:
 double getFootTipElevationMapDifferenceGP(std::string tip);

 //! TODO: Description:
 double getClosestMapValueUsingSpiralIteratorElevation(grid_map::GridMap& MapReference, grid_map::Position footTip, double radius, double tipHeight);

 //! TODO: Description:
 bool supportSurfaceUpperBoundingGP(grid_map::GridMap& upperBoundMap, grid_map::GridMap& supportSurfaceMap);

 //! TODO: Description:
 bool setTerrainVariance(double &terrainVariance);

 //! TODO: Description:
 double getTerrainVariance();

 //! TODO: Description:
 bool setSupportSurfaceUncertaintyEstimation(std::string tip);

 //! TODO: Description:
 double getSupportSurfaceUncertaintyEstimation();

 //! TODO: Description:
 bool footTipElevationMapLayerGP(std::string std);

 //! TODO: Description:
 bool sinkageDepthMapLayerGP(std::string tip, double& tipDifference);


 //! ROS nodehandle.
 ros::NodeHandle nodeHandle_;

 //! Raw elevation map as grid map.
 grid_map::GridMap rawMap_;

 //! Fused elevation map as grid map.
 grid_map::GridMap fusedMap_;

 // New
 //! New grid map for support surface to increase the scope of definition, i.e. fill in holes.
 grid_map::GridMap supportMap_;
 grid_map::GridMap supportMapGP_;
 // End New

 //! Visibility cleanup debug map.
 grid_map::GridMap visibilityCleanupMap_;

 //! Underlying map, used for ground truth maps, multi-robot mapping etc.
 grid_map::GridMap underlyingMap_;

 //! True if underlying map has been set, false otherwise.
 bool hasUnderlyingMap_;

 //! Pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
 kindr::HomTransformQuatD pose_;

 //! ROS publishers.
 ros::Publisher elevationMapRawPublisher_;
 ros::Publisher elevationMapFusedPublisher_;
 ros::Publisher visbilityCleanupMapPublisher_;

 // New
 ros::Publisher elevationMapCorrectedPublisher_;
 ros::Publisher elevationMapSupportPublisher_;
 ros::Publisher elevationMapInpaintedPublisher_;
 ros::Publisher coloredPointCloudPublisher_;
 // End New

 //! Mutex lock for fused map.
 boost::recursive_mutex fusedMapMutex_;

 //! Mutex lock for raw map.
 boost::recursive_mutex rawMapMutex_;

 //! Mutex lock for vsibility cleanup map.
 boost::recursive_mutex visibilityCleanupMapMutex_;

 // NEW TESTING:
 boost::recursive_mutex footTipStanceProcessorMutex_;
 boost::recursive_mutex footTipStanceComparisonMutex_;
 // END NEW!

 //! Underlying map subscriber.
 ros::Subscriber underlyingMapSubscriber_;

 //! Initial ros time
 ros::Time initialTime_;

 //! Parameters. Are set through the ElevationMapping class.
 double minVariance_;
 double maxVariance_;
 double mahalanobisDistanceThreshold_;
 double multiHeightNoise_;
 double minHorizontalVariance_;
 double maxHorizontalVariance_;
 std::string underlyingMapTopic_;
 bool enableVisibilityCleanup_;
 double visibilityCleanupDuration_;
 double scanningDuration_;


 //! Front Feet Positions:
 Eigen::Vector3f LFTipPosition_, RFTipPosition_, LHTipPosition_, RHTipPosition_;
 std::vector<Eigen::Vector3f> LFTipStance_, RFTipStance_, LHTipStance_, RHTipStance_;
 std::vector<bool> processStanceTriggerLeft_, processStanceTriggerRight_, processStanceTriggerLeftHind_, processStanceTriggerRightHind_;
 bool LFTipState_, RFTipState_, LHTipState_, RHTipState_;
 double totalHeightDifference_;
 int heightDifferenceComponentCounter_;
 bool isInStanceLeft_, isInStanceRight_, isInStanceLeftHind_, isInStanceRightHind_;
 std::string comparisonMode_;
 double heightDifferenceFromComparison_;
 double estimatedDriftChange_, estimatedDriftChangeVariance_;
 double oldDiffComparisonUpdate_;
 double estimatedDrift_;
 std::vector<double> weightedDifferenceVector_;

 double usedWeight_;
 bool footTipOutsideBounds_;

 //! Params set upon launching to specify which parts of the program are running.
 //! driftAdjustment_: drift adjustment program running at all.
 //! applyFrameCorrection_: apply the calculated frame Correction -> if false performance measures are still calculated.
 //! runHindLegStanceDetection_: template based stance detection for hind legs for variance estimation in unseen terrain.
 bool driftAdjustment_;
 bool applyFrameCorrection_;
 bool runHindLegStanceDetection_;

 //! Bool to specify wheather in high grass or not:
 bool highGrassMode_;

 //! Tuneable Parameters.
 double kp_, ki_, kd_, weightingFactor_;

 //! For Tuning! To sum up the diffs between the moved elevation map and the foot tips.
 double performanceAssessment_, performanceAssessmentFlat_;

 //! Store values for gras detection
 std::vector<bool> grassDetectionHistory_, grassDetectionHistory2_;

 //! Longer weightedDifferenceVector for PID drift estimation
 std::vector<double> PIDWeightedDifferenceVector_, leftPIDWeightedDifferenceVector_, rightPIDWeightedDifferenceVector_;
 double driftEstimationPID_;

 //! Kalman Filtered Drift Estimation
 double estimatedKalmanDiff_, estimatedKalmanDiffIncrement_, PEstimatedKalmanDiffIncrement_;

 //! Mean foot tip positions
 Eigen::Vector3f meanStance_;

 //! For drift Estimation under Flat ground assumption.
 std::vector<Eigen::Vector3f> leftStanceVector_, rightStanceVector_, leftHindStanceVector_, rightHindStanceVector_;

 //! Life Variance Calculation for roughness characterization in unseen terrain.
 std::vector<double> feetUnseenVarianceVector_;

 //! Message to publish performance assessment values
 //elevation_mapping::PerformanceAssessment performance_assessment_msg_;

 //! Transform Listener and broadcaster for generating the corrected frame
 tf::TransformListener odomDriftAdjustedTransformListener_;
 tf::TransformBroadcaster mapCorrectedOdomTransformBroadcaster_;

 //! ROS subscribers.
 //ros::Subscriber highGrassPointCloudSubscriber_;
 ros::Subscriber footTipStanceSubscriber_;
 //ros::Subscriber highGrassElevationMapSubscriber_;

 //! ROS publishers
 ros::Publisher footContactPublisher_;
 ros::Publisher elevationMapBoundPublisher_;
 ros::Publisher planeFitVisualizationPublisher_;
 ros::Publisher varianceTwistPublisher_;
 ros::Publisher supportSurfaceAddingAreaPublisher_;
 ros::Publisher elevationMapGPPublisher_;

 //! Publication of Markers:
 visualization_msgs::Marker footContactMarkerList_;
 visualization_msgs::Marker elevationMapBoundMarkerList_;
 visualization_msgs::Marker footTipPlaneFitVisualization_;

 //! Coefficients of the foot tip plane fit, calculated in proprioceptive variance estimation and reused in foot tip only elevation map layer..
 Eigen::Vector2f footTipPlaneFitCoefficients_;

 //! Mean values of the three foot tips, used for plane fitting
 Eigen::Vector3f meanOfAllFootTips_;

 bool supportSurfaceInitializationTrigger_;

 //! For use in penetrationDepthVarianceEstimation
 double penetrationDepthVariance_;
 std::vector<double> verticalDifferenceVector_;

 std::string filterChainParametersName_;

 // storage of front left foottip position for simple foot tip embedding.
 Eigen::Vector3f frontLeftFootTip_, frontRightFootTip_;

 // Footprint storage in class member:
 geometry_msgs::Transform footprint_;

 // Variance values.
 double terrainVariance_;
 double PenetrationDepthVariance_;
 double supportSurfaceUncertaintyEstimation_;

 // Foot tip position history for GP.
 std::vector<grid_map::Position3> footTipHistoryGP_;

 // Parameters.
 bool runSupportSurfaceEstimation_;
 bool runHighGrassDetection_;

 // Elevation Map
 ElevationMap map_;

};

} /* namespace */
