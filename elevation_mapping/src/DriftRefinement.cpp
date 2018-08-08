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
    : nodeHandle_(nodeHandle),
      map_(nodeHandle)
{
  elevationMapCorrectedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_drift_adjusted", 1);

  // Launching parameters.
  nodeHandle_.param("apply_frame_correction", applyFrameCorrection_, true);
  nodeHandle_.param("kp", kp_, 0.24);
  nodeHandle_.param("ki", ki_, 0.67);
  nodeHandle_.param("kd", kd_, -0.07);
  nodeHandle_.param("weight_factor", weightingFactor_, 1.0);
  nodeHandle_.param("run_high_grass_detection", runHighGrassDetection_, false);
  nodeHandle_.param("run_support_surface_estimation", runSupportSurfaceEstimation_, false);
  nodeHandle_.param("frame_correction_switching", frameCorrectionSwitching_, false);
  nodeHandle_.param("run_drift_refinement_support_surface", runDriftRefinementSupportSurface_, false);

  //planeFitVisualizationPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("plane_fit_visualization_marker_list", 1000); // DR
  footContactPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("mean_foot_contact_markers_rviz", 1000);
  elevationMapBoundPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("map_bound_markers_rviz", 1000);

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

  // Initialize the markers for publishing the confidence bounds of the fused map and the foot tip positions within them.
  initializeVisualizationMarkers();

  std::cout << "called the drift refinement Constructor" << std::endl;
}

DriftRefinement::~DriftRefinement()
{
}

bool DriftRefinement::footTipElevationMapComparison(std::string tip, Eigen::Vector3f& meanStance, GridMap& rawMap, GridMap& fusedMap, GridMap& supportMap)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceComparisonMutex_);
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipStanceProcessor(footTipStanceProcessorMutex_);
    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

    //std::cout << "Called the foot tip elevation map comparison function" << std::endl;

    // New version
    double xTip, yTip, zTip = 0;
    xTip = meanStance(0);
    yTip = meanStance(1);
    zTip = meanStance(2);

    // Offset of each step versus Elevation map.
    double verticalDifference = 0;
    // Weighted offset depending on the upper and lower bounds of the fused map.
    double weightedVerticalDifferenceIncrement = 0.0;

    // Horizontal position of the foot tip.
    Position tipPosition(xTip, yTip);

    std::cout << "is it inside????? " << rawMap.isInside(tipPosition) << std::endl;

    // Make sure that the state is 1 and the foot tip is inside area covered by the elevation map.
    if(rawMap.isInside(tipPosition) && !isnan(heightDifferenceFromComparison_)){ // HACKED FOR TESTS!!!


        float heightMapRaw = rawMap.atPosition("elevation", tipPosition);


        float varianceMapRaw = rawMap.atPosition("variance", tipPosition);
        float heightMapRawElevationCorrected = heightMapRaw + heightDifferenceFromComparison_; // Changed, since frame transformed grid map used.

        std::cout << "got into the function isnan: " << isnan(heightMapRaw) << " the tip: " << tip << std::endl;

        //Index ind;
        //rawMap_.getIndex(tipPosition, ind);

        //std::cout << "Isvalid?? " << rawMap.isValid(ind) << std::endl;

        // Supress nans.
        if(!isnan(heightMapRaw)){

            std::cout << "got into the function 2: " << heightMapRaw << std::endl;

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
            //Eigen::Array2d ellipseAxes;
            //ellipseAxes[0] = ellipseAxes[1] = std::max(6 * sqrt(rawMap.atPosition("horizontal_variance_x",tipPosition)),
            //                          6 * sqrt(rawMap.atPosition("horizontal_variance_y",tipPosition)));

            // Get lower and upper bound of the fused map.
            auto boundTuple = getFusedCellBounds(tipPosition, fusedMap);
            double lowerBoundFused = std::get<0>(boundTuple);
            double elevationFused = std::get<1>(boundTuple);
            double upperBoundFused = std::get<2>(boundTuple);

            if (runDriftRefinementSupportSurface_) {
                std::cout << "elevation of Support MAP:::::::::::::::::::::::::::::::::::::::::::::::::::::::-> " <<
                             supportMap.atPosition("elevation", tipPosition) << std::endl;
                std::cout << "variance of Support MAP:::::::::::::::::::::::::::::::::::::::::::::::::::::::-> " <<
                             supportMap.atPosition("variance", tipPosition) << std::endl;

                double variance = supportMap.atPosition("variance", tipPosition);

                elevationFused = supportMap.atPosition("elevation", tipPosition);
                lowerBoundFused = elevationFused - 0.002 * sqrt(fabs(variance)); // Testing here..
                upperBoundFused = elevationFused + 0.002 * sqrt(fabs(variance));
                verticalDifference = zTip - supportMap.atPosition("elevation", tipPosition);
                std::cout << "LOWER BOUND BEFORE: " << lowerBoundFused << std::endl;
                std::cout << "UPPER BOUND BEFORE: " << upperBoundFused << std::endl;
                std::cout << "ELEVATION: " << elevationFused << std::endl;
            }

            std::cout << "VERTICAL DIFFERENCE DRIFT ADJUSTMENT: " << verticalDifference << std::endl;

           // std::cout << "lower: " << lowerBoundFused << " elev: " << elevationFused << " upper: " << upperBoundFused << std::endl;
            weightedVerticalDifferenceIncrement = gaussianWeightedDifferenceIncrement(lowerBoundFused, elevationFused, upperBoundFused, verticalDifference);

           // std::cout << "weightedVerticalDifferenceIncrement " << weightedVerticalDifferenceIncrement << std::endl;


            // TODO: Switch ON and OFF
            publishFusedMapBoundMarkers(xTip, yTip, elevationFused, upperBoundFused, lowerBoundFused);

            // TODO: test validity of these:

            // DEBUG
            std::cout << "heightDiff befor weighted difference vector creation: " << heightDifferenceFromComparison_ <<
                         " weightedVerticalDiffIncrement: " << weightedVerticalDifferenceIncrement << std::endl;


            std::cout << "\n \n \n " << std::endl;





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

            //double second_measure_factor = -10000.0;
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


    std::cout << "foot Tip Elev Comparison:  XYXYXYXYXYXYXYXYXY:::::::::::" << heightDifferenceFromComparison_ << std::endl;

    // Publish frame, offset by the height difference parameter.
    //frameCorrection(); // Published here also.. (Hacked)



    // Separate layer:
    //rawMap.add("additional_layer", heightDifferenceFromComparison_);
    //rawMap["elevation_corrected"] = rawMap["elevation_corrected"] + rawMap["additional_layer"];


    // Publish the elevation map with the new layer, at the frequency of the stances.
    grid_map_msgs::GridMap mapMessage;
    GridMapRosConverter::toMessage(rawMap, mapMessage);
    mapMessage.info.header.frame_id = "odom_drift_adjusted"; //! HACKED!!

    //GridMapRosConverter::fromMessage(mapMessage, rawMapCorrected)


    elevationMapCorrectedPublisher_.publish(mapMessage);



    return true;
}

bool DriftRefinement::publishFusedMapBoundMarkers(double& xTip, double& yTip,
                                               double& elevationFused, double& upperBoundFused, double& lowerBoundFused) // SP/DR
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

std::tuple<double, double, double> DriftRefinement::getFusedCellBounds(const Eigen::Vector2d& position, const GridMap& fusedMap)
{
    //boost::recursive_mutex::scoped_lock scopedLockForFootTipComparison(footTipStanceComparisonMutex_);
   // boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
   // boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    float upperFused, lowerFused, elevationFused;
    bool doFuseEachStep = true;
    if(!isnan(heightDifferenceFromComparison_) && doFuseEachStep){
        elevationFused = fusedMap.atPosition("elevation", position);
        lowerFused = fusedMap.atPosition("lower_bound", position);
        upperFused = fusedMap.atPosition("upper_bound", position);
    }
    return std::make_tuple(lowerFused, elevationFused, upperFused);
}

float DriftRefinement::differenceCalculationUsingPID()
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

float DriftRefinement::gaussianWeightedDifferenceIncrement(double lowerBound, double elevation, double upperBound, double diff)
{
    diff -= heightDifferenceFromComparison_;

    std::cout << "LOWER         BOUND: " << lowerBound << std::endl;
    std::cout << "UPPER         BOUND: " << upperBound << std::endl;

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

    if (frameCorrectionSwitching_) {
        // DEBugging:
        if (triggerSum > 3 && runHighGrassDetection_){
          //  std::cout << "<<<<<<<<>>>>>>>>> \n" << "<<<<<<<<<<>>>>>>>> \n" << "<<<<<<<<<<>>>>>>>>> \n";
            highGrassMode_ = true; // Hacked!!!!
            applyFrameCorrection_ = true; // Hacked true into here!!!!!!
        }
        else{
          //  std::cout << "!!!! \n" << "!!!! \n" << "!!!! \n";
            highGrassMode_ = false;
            applyFrameCorrection_ = true;
        }
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


    // Factor, where sigmoid drift correction dropoff starts (weight declines)
    double dropoffFactor = 1.7;


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




    std::cout << "WEIGHT: " << (1-weight) << " diff: " << diff << " elevation: " << elevation <<
                 " lower: " << lowerBound << " upper: " << upperBound << std::endl;
    return (1.0 - weight) * diff;
}

float DriftRefinement::normalDistribution(float arg, float mean, float stdDev)
{
    double e = exp(-pow((arg-mean),2)/(2.0*pow(stdDev,2)));
    //Leaving away the weighting, as the maximum value should be one, which is sensible for weighting.
    return e; // (stdDev * sqrt(2*M_PI));
}

void DriftRefinement::initializeVisualizationMarkers() {
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
}

bool DriftRefinement::publishAveragedFootTipPositionMarkers(const GridMap& rawMap, const Eigen::Vector3f meanStance, std::string tip)
{
    // Positions for publisher.
    geometry_msgs::Point p;
    p.x = meanStance(0);
    p.y = meanStance(1);
    p.z = meanStance(2);

    // Coloring as function of applied weight.
    bool footTipColoring = true;
    double coloring_factor = 2.5;
    std_msgs::ColorRGBA c;
    c.g = 0;
    c.b = max(0.0, 1 - coloring_factor * usedWeight_); // TODO set after tuning to get sensible results..
    c.r = min(1.0, coloring_factor * usedWeight_);
    c.a = 0.5;

    bool hind = false;
    if (tip == "lefthind" || tip == "righthind") hind = true;
    if(hind){
        c.g = 1;
        c.b = 1;
        c.r = 1;
        c.a = 1;
        std::cout << "Hind!!!" << std::endl;
    }

    //boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

    // If outside the area, where comparison can be made (i.e. no elevation map value is found) set black color.
    // Set yellow if foot tip is outside the bounds of the fused map.
    Position coloringPosition(p.x, p.y);
    if (isnan(rawMap.atPosition("elevation", coloringPosition))){
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

} /* namespace */
