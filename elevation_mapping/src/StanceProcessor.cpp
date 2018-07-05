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
      supportSurfaceEstimation_(nodeHandle),
      driftRefinement_(nodeHandle)
{
  std::cout << "called the stanceprocessor Constructor" << std::endl;



  // Launching parameters.
  nodeHandle_.param("run_foot_tip_elevation_map_enhancements", runFootTipElevationMapEnhancements_, true);
  nodeHandle_.param("run_hind_leg_stance_detection", runHindLegStanceDetection_, true);
  nodeHandle_.param("stance_detection_method", stanceDetectionMethod_, string("start"));
  nodeHandle_.param("use_bag", useBag_, false); // SP

  nodeHandle_.param("run_support_surface_estimation", runSupportSurfaceEstimation_, false); // DR, SS??

  //if(runFootTipElevationMapEnhancements_){ TODO: uncomment
      if(!useBag_) footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state", 1, &StanceProcessor::footTipStanceCallback, this);
      else footTipStanceSubscriber_ = nodeHandle_.subscribe("/state_estimator/quadruped_state_remapped", 1, &StanceProcessor::footTipStanceCallback, this);
  //} TODO: uncomment

  // Publish Foot Tip Markers.
  footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000);
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("map_bound_markers_rviz", 1000);
  initializeFootTipMarkers();

  // Initializing Stance Bools.
  isInStanceLeft_ = false;
  isInStanceLeftHind_ = false;
  isInStanceRight_ = false;
  isInStanceRightHind_ = false;
}

StanceProcessor::~StanceProcessor()
{
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

  // Detect start and end of stances for each of the two front foot tips.
  detectStancePhase();

  //frameCorrection();
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
    if (processStanceTriggerLeft_.size() > 2000) {
        processStanceTriggerLeft_.erase(processStanceTriggerLeft_.begin());
    }
    if (processStanceTriggerRight_.size() > 2000) {
        processStanceTriggerRight_.erase(processStanceTriggerRight_.begin());
    }

    // Collect the foot tip position data (if foot tip in contact). (Prevent nans)
    if (LFTipState_ && isInStanceLeft_) {
        LFTipStance_.push_back(LFTipPosition_);
    }
    // For start detection scheme.
    else if (LFTipState_ && stanceDetectionMethod_ == "start") LFTipStance_.push_back(LFTipPosition_);
    if (RFTipState_ && isInStanceRight_) {
        RFTipStance_.push_back(RFTipPosition_);
    }
    else if (RFTipState_ && stanceDetectionMethod_ == "start") RFTipStance_.push_back(RFTipPosition_);

    // Stance Detection
    templateMatchingForStanceDetection("left", processStanceTriggerLeft_);
    templateMatchingForStanceDetection("right", processStanceTriggerRight_);

    // Here again for hind legs..
    if (runHindLegStanceDetection_) {

        // Collect State Data for stance phase detection
        processStanceTriggerLeftHind_.push_back(LHTipState_);
        processStanceTriggerRightHind_.push_back(RHTipState_);

        // Constrain the size of the state arrays.
        if (processStanceTriggerLeftHind_.size() > 2000) {
            processStanceTriggerLeftHind_.erase(processStanceTriggerLeftHind_.begin());
        }
        if (processStanceTriggerRightHind_.size() > 2000) {
            processStanceTriggerRightHind_.erase(processStanceTriggerRightHind_.begin());
        }

        // Collect the foot tip position data (if foot tip in contact). (Prevent nans)
        if (LHTipState_ && isInStanceLeftHind_) {
            LHTipStance_.push_back(LHTipPosition_);
        }
        else if (LHTipState_ && stanceDetectionMethod_ == "start") LHTipStance_.push_back(LHTipPosition_);

        if (RHTipState_ && isInStanceRightHind_) {
            RHTipStance_.push_back(RHTipPosition_);
        }
        else if (RHTipState_ && stanceDetectionMethod_ == "start") RHTipStance_.push_back(RHTipPosition_);

        // Stance Detection
        templateMatchingForStanceDetection("lefthind", processStanceTriggerLeftHind_);
        templateMatchingForStanceDetection("righthind", processStanceTriggerRightHind_);
    }
    return true;
}

bool StanceProcessor::templateMatchingForStanceDetection(std::string tip, std::vector<bool> &stateVector)
{
    // Recognition of the start of a stance phase.
    if(stateVector.size() >= 16 && stateVector.end()[-1]+stateVector.end()[-2]+
            stateVector.end()[-3]+stateVector.end()[-4]+stateVector.end()[-5]+
            stateVector.end()[-6]+stateVector.end()[-7]+stateVector.end()[-8] >= 6 &&
            stateVector.end()[-9]+stateVector.end()[-10] +
            stateVector.end()[-11]+stateVector.end()[-12]+ stateVector.end()[-13]+
            stateVector.end()[-14]+stateVector.end()[-15] <= 6){
        if(tip == "left" && !isInStanceLeft_){
           // std::cout << "Start of LEFT stance" << std::endl;
            isInStanceLeft_ = 1;
            if (stanceDetectionMethod_ == "start") if(!processStance(tip)) return false;
        }
        if(tip == "lefthind" && !isInStanceLeftHind_){
            isInStanceLeftHind_ = 1;
            if (stanceDetectionMethod_ == "start") if(!processStance(tip)) return false;
        }
        if(tip == "right" && !isInStanceRight_){
           // std::cout << "Start of Right stance" << std::endl;
            isInStanceRight_ = 1;
            if (stanceDetectionMethod_ == "start") if(!processStance(tip)) return false;
        }
        if(tip == "righthind" && !isInStanceRightHind_){
            isInStanceRightHind_ = 1;
            if (stanceDetectionMethod_ == "start") if(!processStance(tip)) return false;
        }
    }

    // Recognition of the end of a stance phase.
    if(stateVector.size() >= 16 &&
            stateVector.end()[-1] + stateVector.end()[-2] + stateVector.end()[-3]+
            stateVector.end()[-4]+stateVector.end()[-5] + stateVector.end()[-6]+
            stateVector.end()[-7]+stateVector.end()[-8] <= 3 &&
            stateVector.end()[-9]+stateVector.end()[-10] +
            stateVector.end()[-11]+stateVector.end()[-12]+ stateVector.end()[-13]+
            stateVector.end()[-14]+stateVector.end()[-15] >=1){

        if(tip == "left" && isInStanceLeft_){
            if (stanceDetectionMethod_ == "average") if(!processStance(tip)) return false;
            isInStanceLeft_ = 0;
            processStanceTriggerLeft_.clear();
        }
        if(tip == "lefthind" && isInStanceLeftHind_){
            if (stanceDetectionMethod_ == "average") if(!processStance(tip)) return false;
            isInStanceLeftHind_ = 0;
            processStanceTriggerLeftHind_.clear();
        }
        if(tip == "right" && isInStanceRight_){
            if (stanceDetectionMethod_ == "average") if(!processStance(tip)) return false;
            isInStanceRight_ = 0;
            processStanceTriggerRight_.clear();
        }
        if(tip == "righthind" && isInStanceRightHind_){
            if (stanceDetectionMethod_ == "average") if(!processStance(tip)) return false;
            isInStanceRightHind_ = 0;
            processStanceTriggerRightHind_.clear();
        }
    }
    return true;
}

bool StanceProcessor::processStance(std::string tip)
{

//    // Delete the last 10 entries of the Foot Stance Position Vector, as these are used for transition detection
    if (stanceDetectionMethod_ == "start") deleteFirstEntriesOfStances(tip);
    if (stanceDetectionMethod_ == "average") deleteLastEntriesOfStances(tip);

    Eigen::Vector3f meanStance = getAverageFootTipPositions(tip);

//    bool runProprioceptiveRoughnessEstimation = true;
//    if(runProprioceptiveRoughnessEstimation) proprioceptiveRoughnessEstimation(tip);

    bool hind = false;
    if(tip != "lefthind" && tip != "righthind") driftRefinement_.footTipElevationMapComparison(tip, meanStance);
    else hind = true;
    publishAveragedFootTipPositionMarkers(hind);

//    // Performance Assessment, sensible if wanting to tune the system while walking on flat surface.
//    //bool runPreformanceAssessmentForFlatGround = false;
//    //if(runPreformanceAssessmentForFlatGround) performanceAssessmentMeanElevationMap();

    std::cout << "Process Stance: " << tip << std::endl;

    return true;
}

bool StanceProcessor::deleteFirstEntriesOfStances(std::string tip)
{
    // All stance entries are deleted except the last 3 ones are stored -> this gives start detection
    if (tip == "left" && LFTipStance_.size() < 3){
        std::cout << "WARNING: LEFT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (tip == "right" && RFTipStance_.size() < 3){
        std::cout << "WARNING: RIGHT STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (tip == "lefthind" && LHTipStance_.size() < 3){
        std::cout << "WARNING: LEFT HIND STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else if (tip == "righthind" && RHTipStance_.size() < 3){
        std::cout << "WARNING: RIGHT HIND STANCE PHASE HAS TOO LITTLE ENTRIES TO BE PROCESSED" << std::endl;
    }
    else{
        if (tip == "left"){
            std::vector<Eigen::Vector3f> newLFTipStance;
            for (unsigned int j = 1; j <= 3; ++j) {
                newLFTipStance.push_back(LFTipStance_[LFTipStance_.size() - j]);     // Probably an additional -1 is needed
            }
            LFTipStance_.clear();
            LFTipStance_ = newLFTipStance;
            //LFTipStance_.erase(LFTipStance_.begin()); // pop_front (.erase(begin)) // TODO
        }
        if (tip == "right"){
            std::vector<Eigen::Vector3f> newRFTipStance;
            for (unsigned int j = 1; j <= 3; ++j) {
                newRFTipStance.push_back(RFTipStance_[RFTipStance_.size() - j]);     // Probably an additional -1 is needed
            }
            RFTipStance_.clear();
            RFTipStance_ = newRFTipStance;
        }
        if (tip == "lefthind"){
            std::vector<Eigen::Vector3f> newLHTipStance;
            for (unsigned int j = 1; j <= 3; ++j) {
                newLHTipStance.push_back(LHTipStance_[LHTipStance_.size() - j]);     // Probably an additional -1 is needed
            }
            LHTipStance_.clear();
            LHTipStance_ = newLHTipStance;
        }
        if (tip == "righthind"){
            std::vector<Eigen::Vector3f> newRHTipStance;
            for (unsigned int j = 1; j <= 3; ++j) {
                newRHTipStance.push_back(RHTipStance_[RHTipStance_.size() - j]);     // Probably an additional -1 is needed
            }
            RHTipStance_.clear();
            RHTipStance_ = newRHTipStance;
        }
    }
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

Eigen::Vector3f StanceProcessor::getAverageFootTipPositions(std::string tip)
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

    // Save the mean tip positions for simple foot tip embedding into high grass detection
    if (tip == "left") frontLeftFootTip_ = meanStance_;
    if (tip == "right") frontRightFootTip_ = meanStance_;
    if (tip == "lefthind") hindLeftFootTip_ = meanStance_;
    if (tip == "righthind") hindRightFootTip_ = meanStance_;

    return meanStance_;
}

bool StanceProcessor::publishAveragedFootTipPositionMarkers(bool hind)
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
    //if (isnan(map_.rawMap_.atPosition("elevation", coloringPosition))){
        c.g = 0;
        c.b = 0;
        c.r = 0;
    //}
    //else if(footTipOutsideBounds_) c.g = 0.9;

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

    // TODO: get the values from the drift adjustment and publish the map bound markers.

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

} /* namespace */
