/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/StanceProcessor.hpp"

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

StanceProcessor::StanceProcessor(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle),
      //rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy",
      //        "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan",
      //        "sensor_z_at_lowest_scan", "foot_tip_elevation", "support_surface", "elevation_inpainted", "elevation_smooth", "vegetation_height", "vegetation_height_smooth",
      //        "support_surface", "support_surface_smooth", "support_surface_added"}),//, "support_surface_smooth_inpainted", "support_surface_added"}),
      //fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      //supportMap_({"elevation", "variance", "elevation_gp", "elevation_gp_added"}), // New
      //supportMapGP_({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}), // New
      //hasUnderlyingMap_(false),
      //visibilityCleanupDuration_(0.0),
      filterChain_("grid_map::GridMap"), // New
      filterChain2_("grid_map::GridMap")
{
    // Timon added foot_tip_elevation layer
  //rawMap_.setBasicLayers({"elevation", "variance"});
  //fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  //supportMap_.setBasicLayers({"elevation", "variance", "elevation_gp", "elevation_gp_added"}); // New
  //supportMapGP_.setBasicLayers({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}); // New

  //! uncomment!!
  //clear();


  std::cout << "called the stanceprocessor Constructor" << std::endl;

  // TEST:
  elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);
  elevationMapSupportPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_added", 1);
  elevationMapInpaintedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface", 1);
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
  //nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("/grid_map_filter_chain_one"));
  //if(!filterChain_.configure("/grid_map_filter_chain_one", nodeHandle_)){
  //    std::cout << "Could not configure the filter chain!!" << std::endl;
  //    return;
  //}
  //if(!filterChain2_.configure("/grid_map_filter_chain_two", nodeHandle_)){
  //    std::cout << "INBETWEEN Prob" << std::endl;
  //    std::cout << "Could not configure the filter chain!!" << std::endl;
  //    return;
  //}

  bool use_bag = true;

  // (New:) Foot tip position Subscriber for Foot tip - Elevation comparison
  // TESTED BY CHANGING IF SCOPES..


  //! Uncomment:
  if(driftAdjustment_){
      if(!use_bag) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &StanceProcessor::footTipStanceCallback, this);
      else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 1, &StanceProcessor::footTipStanceCallback, this);
  }

  // NEW: publish foot tip markers
  footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000);
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("map_bound_markers_rviz", 1000);
  planeFitVisualizationPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("plane_fit_visualization_marker_list", 1000);
  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);
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

  //initialTime_ = ros::Time::now();
}

StanceProcessor::~StanceProcessor()
{
}

void StanceProcessor::footTipStanceCallback(const quadruped_msgs::QuadrupedState& quadrupedState)
{

  //std::cout << "Calling Back still!!! STANCE PROCESSOR" << std::endl;

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
  //setFootprint(quadrupedState.frame_transforms[3].transform);


  // Check if walking forward or backwards. TODO!
  //(double)quadrupedState.twist.twist.linear.x;

  // Detect start and end of stances for each of the two front foot tips.
  detectStancePhase();
  //detectStancePhase("right");
  frameCorrection();



}

bool StanceProcessor::detectStancePhase()
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

bool StanceProcessor::templateMatchingForStanceDetection(std::string tip, std::vector<bool> &stateVector)
{
    //std::cout << "tip: " << tip << std::endl;
    //std::cout << "statevec size: " << stateVector.size() << std::endl;
    //std::cout << isInStanceLeftHind_ << std::endl;


    // Recognition of the start of a stance phase of the front legs.
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


    // Recognition of the end of a stance phase of the front legs.
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

bool StanceProcessor::processStance(std::string tip)
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

bool StanceProcessor::deleteLastEntriesOfStances(std::string tip)
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

bool StanceProcessor::getAverageFootTipPositions(std::string tip)
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

bool StanceProcessor::publishAveragedFootTipPositionMarkers(bool hind)
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
    if (isnan(map_.rawMap_.atPosition("elevation", coloringPosition))){
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

bool StanceProcessor::publishFusedMapBoundMarkers(double& xTip, double& yTip,
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

bool StanceProcessor::footTipElevationMapComparison(std::string tip)
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

        //! commented this. -> at some point: create object of support surface estimation..
        //if(rawMap_.isInside(tipPosition)) updateSupportSurfaceEstimation(tip); // NEW !!!!!
        //else std::cout << "FOOT TIP CONSIDERED NOT TO BE INSIDE!!!!! \n \n \n \n " << std::endl;

        // Make sure that the state is 1 and the foot tip is inside area covered by the elevation map.
        if(map_.rawMap_.isInside(tipPosition) && !isnan(heightDifferenceFromComparison_)){ // HACKED FOR TESTS!!!
            float heightMapRaw = map_.rawMap_.atPosition("elevation", tipPosition);
            float varianceMapRaw = map_.rawMap_.atPosition("variance", tipPosition);
            float heightMapRawElevationCorrected = map_.rawMap_.atPosition("elevation", tipPosition) + heightDifferenceFromComparison_; // Changed, since frame transformed grid map used.

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
                ellipseAxes[0] = ellipseAxes[1] = std::max(6 * sqrt(map_.rawMap_.atPosition("horizontal_variance_x",tipPosition)),
                                          6 * sqrt(map_.rawMap_.atPosition("horizontal_variance_y",tipPosition)));

                // Get lower and upper bound of the fused map.
                auto boundTuple = getFusedCellBounds(tipPosition, ellipseAxes);
                double lowerBoundFused = std::get<0>(boundTuple);
                double elevationFused = std::get<1>(boundTuple);
                double upperBoundFused = std::get<2>(boundTuple);
               // std::cout << "lower: " << lowerBoundFused << " elev: " << elevationFused << " upper: " << upperBoundFused << std::endl;
                weightedVerticalDifferenceIncrement = gaussianWeightedDifferenceIncrement(lowerBoundFused, elevationFused, upperBoundFused, verticalDifference);

               // std::cout << "weightedVerticalDifferenceIncrement " << weightedVerticalDifferenceIncrement << std::endl;


                //bool runPenetrationDepthVarianceEstimation = true;
                //if (runPenetrationDepthVarianceEstimation) penetrationDepthVarianceEstimation(tip, verticalDifference);

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

    std::cout << "foot tip comparison done .. " << std::endl;



    // Publish the elevation map with the new layer, at the frequency of the stances.
    grid_map_msgs::GridMap mapMessage;
    GridMapRosConverter::toMessage(map_.rawMap_, mapMessage);
    mapMessage.info.header.frame_id = "odom_drift_adjusted"; //! HACKED!!

    //GridMapRosConverter::fromMessage(mapMessage, rawMapCorrected)

    elevationMapCorrectedPublisher_.publish(mapMessage);



    return true;
}

bool StanceProcessor::initializeFootTipMarkers()
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

std::tuple<double, double, double> StanceProcessor::getFusedCellBounds(const Eigen::Vector2d& position, const Eigen::Array2d& length)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipComparison(footTipStanceComparisonMutex_);
   // boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
   // boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    float upperFused, lowerFused, elevationFused;
    bool doFuseEachStep = true;
    if(!isnan(heightDifferenceFromComparison_) && doFuseEachStep){
        map_.fuseArea(position, length); // HAcked object..
        elevationFused = map_.fusedMap_.atPosition("elevation", position);
        lowerFused = map_.fusedMap_.atPosition("lower_bound", position);
        upperFused = map_.fusedMap_.atPosition("upper_bound", position);
    }
    return std::make_tuple(lowerFused, elevationFused, upperFused);
}

bool StanceProcessor::frameCorrection()
{
    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
    //boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);

   // std::cout << "entered the frame correction" << std::endl;


    // Transform Broadcaster for the /odom_z_corrected frame.
    tf::Transform odomMapTransform;

    odomMapTransform.setIdentity();

    if (!isnan(heightDifferenceFromComparison_)) odomMapTransform.getOrigin()[2] += heightDifferenceFromComparison_;
    else std::cout << heightDifferenceFromComparison_ << " <- height diff is this kind of NAN for some reason? \n ? \n ? \n";

    //ros::Time stamp = ros::Time().fromNSec(fusedMap_.getTimestamp());

    //std::cout << "TIMESTAMP PUBLISHED THE odom_drift_adjusted TRANSFORM!!: " << stamp << std::endl;

    mapCorrectedOdomTransformBroadcaster_.sendTransform(tf::StampedTransform(odomMapTransform,
                                          ros::Time().fromNSec(map_.rawMap_.getTimestamp()), "odom", "odom_drift_adjusted"));

    return true;
}

float StanceProcessor::differenceCalculationUsingPID()
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

bool StanceProcessor::setFootprint(const geometry_msgs::Transform& footprint){
    footprint_ = footprint;
    return true;
}

bool StanceProcessor::proprioceptiveRoughnessEstimation(std::string tip){


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

bool StanceProcessor::proprioceptiveVariance(std::string tip){

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

            // set function..
            setTerrainVariance(varianceMsg.linear.z);

            //varianceMsg.angular.x = getPenetrationDepthVariance();
           // footTipPlaneFitVisualization_.action = visualization_msgs::Marker::;

            // TODO: Do visualize fitted Plane!!
            //varianceTwistPublisher_.publish(varianceMsg);
        }

    }
    // TODO: add Layer, that takes variace uncertainty into account.
}

float StanceProcessor::gaussianWeightedDifferenceIncrement(double lowerBound, double elevation, double upperBound, double diff)
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

float StanceProcessor::normalDistribution(float arg, float mean, float stdDev)
{
    double e = exp(-pow((arg-mean),2)/(2.0*pow(stdDev,2)));
    //Leaving away the weighting, as the maximum value should be one, which is sensible for weighting.
    return e; // (stdDev * sqrt(2*M_PI));
}

bool StanceProcessor::setTerrainVariance(double& terrainVariance){
    terrainVariance_ = terrainVariance;
}

void StanceProcessor::setFootTipPlaneFitCoefficients(Eigen::Vector2f& coeffs){
    footTipPlaneFitCoefficients_ = coeffs;
}

void StanceProcessor::setMeanOfAllFootTips(Eigen::Vector3f& mean){
    meanOfAllFootTips_ = mean;
}

} /* namespace */
