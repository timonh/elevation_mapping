/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/SupportSurfaceEstimation.hpp"
#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
//#include "elevation_mapping/ElevationMapFunctors.hpp"
//#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
//#include "elevation_mapping/HighGrassElevationMapping.hpp"

//// Grid Map
//#include <grid_map_msgs/GridMap.h>
//#include <grid_map_core/grid_map_core.hpp> // New for high grass
//#include <grid_map_filters/BufferNormalizerFilter.hpp>
//#include <grid_map_cv/grid_map_cv.hpp> // New for high grass

//#include <grid_map_cv/InpaintFilter.hpp> // New for high grass
//#include <opencv/cv.h>
//#include <grid_map_filters/ThresholdFilter.hpp>

//#include <grid_map_cv/InpaintFilter.hpp>
//#include <pluginlib/class_list_macros.h>
//#include <ros/ros.h>

//#include <grid_map_cv/GridMapCvConverter.hpp>
//#include <grid_map_cv/GridMapCvProcessing.hpp>

//// TEST
//#include <any_msgs/State.h>

//// GP Regression
//#include <gaussian_process_regression/gaussian_process_regression.h>

//// Math
//#include <math.h>

//// ROS Logging
//#include <ros/ros.h>

//// Eigen
//#include <Eigen/Dense>

//// TEST thread
//#include <thread>

//// PCL conversions
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/transforms.h>

//// PCL normal estimation
//#include <pcl/features/normal_3d.h>

// ROS msgs
//#include <elevation_mapping/PerformanceAssessment.h>

// File IO
//#include <iostream>
//#include <fstream>

using namespace std;
using namespace grid_map;

namespace elevation_mapping {

SupportSurfaceEstimation::SupportSurfaceEstimation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      //rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy",
      //        "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan",
      //        "sensor_z_at_lowest_scan", "foot_tip_elevation", "support_surface", "elevation_inpainted", "elevation_smooth", "vegetation_height", "vegetation_height_smooth",
      //        "support_surface", "support_surface_smooth", "support_surface_added"}),//, "support_surface_smooth_inpainted", "support_surface_added"}),
      //fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      //supportMap_({"elevation", "variance", "elevation_gp", "elevation_gp_added"}), // New
      supportMapGP_({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}), // New
      //hasUnderlyingMap_(false),
      //visibilityCleanupDuration_(0.0),
      filterChain_("grid_map::GridMap"), // New
      filterChain2_("grid_map::GridMap")
{
    // Timon added foot_tip_elevation layer
  //rawMap_.setBasicLayers({"elevation", "variance"});
  //fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  //supportMap_.setBasicLayers({"elevation", "variance", "elevation_gp", "elevation_gp_added"}); // New
  supportMapGP_.setBasicLayers({"elevation_gp", "variance_gp", "elevation_gp_added", "elevation_gp_tip", "sinkage_depth_gp"}); // New

  //! uncomment!!
  //clear();

  elevationMapSupportPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_added", 1); // SS
  elevationMapInpaintedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface", 1); // SS
  elevationMapGPPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface_gp", 1); // SS

  nodeHandle_.param("add_old_support_surface_data_to_gp_training", addOldSupportSurfaceDataToGPTraining_, false); // SS
  nodeHandle_.param("weight_terrain_continuity", weightTerrainContinuity_, 1.0); // SS
  nodeHandle_.param("run_terrain_continuity_biasing", runTerrainContinuityBiasing_, true); // SS
  nodeHandle_.param("exponent_sinkage_depth_weight", exponentSinkageDepthWeight_, 2.0); // SS
  nodeHandle_.param("exponent_terrain_continuity_weight", exponentTerrainContinuityWeight_, 2.0); // SS
  nodeHandle_.param("weight_decay_threshold", weightDecayThreshold_, 0.4); // SS
  nodeHandle_.param("exponent_characteristic_value", exponentCharacteristicValue_, 1.0); // SS
  nodeHandle_.param("continuity_filter_gain", continuityFilterGain_, 0.3); // SS

  // For filter chain. // SS
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

  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);
  supportSurfaceAddingAreaPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("adding_area", 1000);


  supportSurfaceInitializationTrigger_ = false; // SS
  cumulativeSupportSurfaceUncertaintyEstimation_ = 0.0; // SS

  initialTime_ = ros::Time::now();
}

SupportSurfaceEstimation::~SupportSurfaceEstimation()
{
}

bool SupportSurfaceEstimation::updateSupportSurfaceEstimation(std::string tip, GridMap& rawMap){

    // Uncertainty Estimation based on low pass filtered error between foot tip height and predicted support Surface.
    setSupportSurfaceUncertaintyEstimation(tip);

    // Set coefficients spun by three of the foot tips.
    //terrainContinuityBiasing(tip);

    // TODO: these params into config file.
    double tileResolution = 0.08;
    double tileDiameter = 0.23; // HAcked larger, if slow, change this here
    double sideLengthAddingPatch = 1.3;
    //mainGPRegression(tileResolution, tileDiameter, sideLengthAddingPatch, tip);
    return true;
}

Position3 SupportSurfaceEstimation::getFrontLeftFootTipPosition(){
    Position3 tipPos(frontLeftFootTip_(0), frontLeftFootTip_(1), frontLeftFootTip_(2));
    return tipPos;
}

Position3 SupportSurfaceEstimation::getFrontRightFootTipPosition(){
    Position3 tipPos(frontRightFootTip_(0), frontRightFootTip_(1), frontRightFootTip_(2));
    return tipPos;
}

Position3 SupportSurfaceEstimation::getHindLeftFootTipPosition(){
    Position3 tipPos(hindLeftFootTip_(0), hindLeftFootTip_(1), hindLeftFootTip_(2));
    return tipPos;
}

Position3 SupportSurfaceEstimation::getHindRightFootTipPosition(){
    Position3 tipPos(hindRightFootTip_(0), hindRightFootTip_(1), hindRightFootTip_(2));
    return tipPos;
}

bool SupportSurfaceEstimation::setSupportSurfaceUncertaintyEstimation(std::string tip){

//    Eigen::Vector3f latestTip;
//    if (tip == "left") latestTip = getLatestLeftStance();
//    if (tip == "right") latestTip = getLatestRightStance();

//    // So far not low pass filtered!! -> create Vector for that (TODO)
//    grid_map::Position latestTipPosition(latestTip(0), latestTip(1));
//    double diff = fabs(latestTip(2) - supportMapGP_.atPosition("elevation_gp_added", latestTipPosition));
//    supportSurfaceUncertaintyEstimation_ = diff;
//    if (!isnan(diff)) cumulativeSupportSurfaceUncertaintyEstimation_ += diff;
    return true;
}

double SupportSurfaceEstimation::getFootTipElevationMapDifferenceGP(std::string tip){

    double radius = 0.1; // Maximum search radius for spiralling search in order to find the closest map element in case if nan is present..
    Position3 footTip3;
    if (tip == "left") footTip3 = getFrontLeftFootTipPosition();
    if (tip == "right") footTip3 = getFrontRightFootTipPosition();
    Position footTipHorizontal(footTip3(0), footTip3(1));
    double verticalDifference;
    if (supportMapGP_.exists("smoothed_top_layer_gp")) { // New Check!!!
        if(supportMapGP_.isInside(footTipHorizontal)){
            if (!isnan(supportMapGP_.atPosition("smoothed_top_layer_gp", footTipHorizontal))){
                verticalDifference = footTip3(2) - supportMapGP_.atPosition("smoothed_top_layer_gp", footTipHorizontal); // Hacked to rawMap_
            }
            else verticalDifference = getClosestMapValueUsingSpiralIteratorElevation(supportMapGP_, footTipHorizontal, radius, footTip3(2)); // New experiment.. Wrong, not difference yet!!!
        }
    }
    else verticalDifference = std::numeric_limits<double>::quiet_NaN();
    std::cout << "Vertical Difference!! -> ->: " << verticalDifference << std::endl;
    return verticalDifference;
}

double SupportSurfaceEstimation::getClosestMapValueUsingSpiralIteratorElevation(grid_map::GridMap& MapReference, Position footTip, double radius, double tipHeight){

    int counter = 0;
    for (grid_map::SpiralIterator iterator(MapReference, footTip, radius);
         !iterator.isPastEnd(); ++iterator) {   // Hacked to is inside..
        Index index(*iterator);
        Position pos;
        MapReference.getPosition(index, pos);
        if (MapReference.isInside(pos) && !isnan(MapReference.at("smoothed_top_layer_gp", index)) && MapReference.isValid(index)){
            std::cout << "RETURNED DIFFERENCE TO A CLOSE NEIGHBOR USING SPIRALLING!!!" << std::endl;
            return tipHeight - MapReference.at("smoothed_top_layer_gp", index); // && MapReference.isValid(index)
        }
        counter++;
        if (counter > 28) break;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool SupportSurfaceEstimation::penetrationDepthVarianceEstimation(std::string tip, double verticalDifference){

    int maxHistory = 3;

    if (!isnan(verticalDifference)) verticalDifferenceVector_.push_back(verticalDifference);
    if (verticalDifferenceVector_.size() > maxHistory) verticalDifferenceVector_.erase(verticalDifferenceVector_.begin()); // Hacking around with the length of the vector.
    double totalVerticalDifference = 0.0;
    double squaredTotalVerticalDifference = 0.0;
    double totalVerticalDifferenceChange = 0.0;
    double squaredTotalVerticalDifferenceChange = 0.0;

    int count = verticalDifferenceVector_.size();
    for (auto& n : verticalDifferenceVector_) {
        totalVerticalDifference += n;
        squaredTotalVerticalDifference += pow(n, 2);
    }

    // Differencial version.
    if (verticalDifferenceVector_.size() > 1) {
        for (unsigned int j = 1; j < verticalDifferenceVector_.size(); ++j) {
            totalVerticalDifferenceChange += verticalDifferenceVector_[j] - verticalDifferenceVector_[j-1];
            squaredTotalVerticalDifferenceChange += pow(verticalDifferenceVector_[j] - verticalDifferenceVector_[j-1], 2);
        }
    }

    double penetrationDepthVariance = squaredTotalVerticalDifference / double(count) -
            pow(totalVerticalDifference / double(count), 2);

    // Differential Version.
    double differentialPenetrationDepthVariance = squaredTotalVerticalDifferenceChange / double(count) -
            pow(totalVerticalDifferenceChange / double(count), 2);

    setPenetrationDepthVariance(penetrationDepthVariance);
    setDifferentialPenetrationDepthVariance(differentialPenetrationDepthVariance);

    std::cout << "differentialPenetrationDepthVariance_: " << differentialPenetrationDepthVariance_ << std::endl;

    return true;
}

// For using it in the penetration depth estimation.
void SupportSurfaceEstimation::setPenetrationDepthVariance(double penetrationDepthVariance){
    if (!isnan(penetrationDepthVariance)) penetrationDepthVariance_ = penetrationDepthVariance;
}

// For using it in the penetration depth estimation.
void SupportSurfaceEstimation::setDifferentialPenetrationDepthVariance(double differentialPenetrationDepthVariance){
    if (!isnan(differentialPenetrationDepthVariance)) differentialPenetrationDepthVariance_ = differentialPenetrationDepthVariance;
}

bool SupportSurfaceEstimation::setSmoothenedTopLayer(std::string tip, GridMap& rawMap){

    // Smoothing radius of area considered for sinkage depth calculation
    double smoothingRadius = 0.15;

    // Get the foot tip position.
    Eigen::Vector3f tipVec;
    if (tip == "left") tipVec = getLatestLeftStance();
    if (tip == "right") tipVec = getLatestRightStance();
    Position footTipPosition(tipVec(0), tipVec(1));

    // Gaussian Process Regression Parameters.
    int inputDim = 1;
    int outputDim = 2;
    GaussianProcessRegression<float> smoothedTopLayerGPR(inputDim, outputDim);
    smoothedTopLayerGPR.SetHyperParams(2.0, 0.1, 0.001);

    std::cout << "just some check...... 0" << std::endl;

    // Iterate around the foot tip position to add Data
    for (CircleIterator iterator(supportMapGP_, footTipPosition, smoothingRadius); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        // Loop Here for adding data..
        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);

        std::cout << "just some check...... 1" << std::endl;

        Position inputPosition;
        supportMapGP_.getPosition(index, inputPosition);
        std::cout << "just some check...... 3" << std::endl;
        trainInput(0) = inputPosition(0);
        trainInput(1) = inputPosition(1);
        std::cout << "just some check...... 4" << std::endl;
        trainOutput(0) = rawMap.at("elevation", index);
        std::cout << "just some check...... 2" << std::endl;

        if (!isnan(trainOutput(0)) && rawMap.isValid(index)){
            std::cout << "just some check...... 5" << std::endl;
            smoothedTopLayerGPR.AddTrainingData(trainInput, trainOutput);
        }
    }

    std::cout << "just some check......" << std::endl;

    if (smoothedTopLayerGPR.get_n_data() > 0){
        for (CircleIterator iterator(supportMapGP_, footTipPosition, smoothingRadius); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);

            Eigen::VectorXf testInput(inputDim);
            Eigen::VectorXf testOutput(outputDim);
            Position testPosition;
            supportMapGP_.getPosition(index, testPosition);
            testInput(0) = testPosition(0);
            testInput(1) = testPosition(1);

            double outputHeight = smoothedTopLayerGPR.DoRegression(testInput)(0);

            if (!isnan(outputHeight)){
                supportMapGP_.at("smoothed_top_layer_gp", index) = outputHeight;
            }
        }
    }
    return true;
}

Eigen::Vector3f SupportSurfaceEstimation::getLatestLeftStance(){
    return leftStanceVector_[leftStanceVector_.size()-1];
}

Eigen::Vector3f SupportSurfaceEstimation::getLatestRightStance(){
    return rightStanceVector_[rightStanceVector_.size()-1];
}

bool SupportSurfaceEstimation::proprioceptiveRoughnessEstimation(std::string tip){

    //! TODO: make function: update stance vectors.
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
    proprioceptiveVariance(tip);
}

bool SupportSurfaceEstimation::proprioceptiveVariance(std::string tip){

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



//    //    for all 4 feet..
//    // TODO: Plane Fitting for the 4 foot tips to get deviation from it (formulate as variance if squared difference, is that valid?)
//    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0 &&
//            leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0){ // Check, that at least one stance position per foot tip is available.
//        Eigen::Vector3f leftTip, rightTip, leftHindTip, rightHindTip;
//        leftTip = leftStanceVector_[leftStanceVector_.size()-1];
//        rightTip = rightStanceVector_[rightStanceVector_.size()-1];
//        leftHindTip = leftHindStanceVector_[leftHindStanceVector_.size()-1];
//        rightHindTip = rightHindStanceVector_[rightHindStanceVector_.size()-1];

//        double meanZelevation = (leftTip(2) + rightTip(2) + leftHindTip(2) + rightHindTip(2)) / 4.0;
//    //    std::cout << "meanzvaluefrom foottips: " << meanZelevation << std::endl;

//        Eigen::Matrix2f leftMat;
//        leftMat(0, 0) = pow(leftTip(0),2)+pow(rightTip(0),2) + pow(leftHindTip(0),2)+pow(rightHindTip(0),2);
//        leftMat(1, 0) = leftMat(0, 1) = leftTip(0) * leftTip(1) + rightTip(0) * rightTip(1) +
//                leftHindTip(0) * leftHindTip(1) + rightHindTip(0) * rightHindTip(1);
//        leftMat(1, 1) = pow(leftTip(1),2)+pow(rightTip(1),2) + pow(leftHindTip(1),2)+pow(rightHindTip(1),2);
//        Eigen::Vector2f rightVec;
//        rightVec(0) = leftTip(0) * (leftTip(2)-meanZelevation) + rightTip(0) * (rightTip(2)-meanZelevation) +
//                leftHindTip(0) * (leftHindTip(2)-meanZelevation) + rightHindTip(0) * (rightHindTip(2)-meanZelevation);
//        rightVec(1) = leftTip(1) * (leftTip(2)-meanZelevation) + rightTip(1) * (rightTip(2)-meanZelevation) +
//                leftHindTip(1) * (leftHindTip(2)-meanZelevation) + rightHindTip(1) * (rightHindTip(2)-meanZelevation);

//        Eigen::Vector2f sol = leftMat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-rightVec);

//        setFootTipPlaneFitCoefficients(sol);
//        Eigen::Vector3f meanOfAllFootTips;
//        for (unsigned int j = 0; j <= 2; ++j) meanOfAllFootTips(j) = 0.25 * (leftTip(j) + rightTip(j) + leftHindTip(j) + rightHindTip(j));
//        setMeanOfAllFootTips(meanOfAllFootTips);


//        bool visualizeFootTipPlaneFit = true;
//        if (visualizeFootTipPlaneFit){

//            geometry_msgs::Point p1, p2, p3, p4;
//            p1.x = leftTip(0);
//            p2.x = rightTip(0);
//            p3.x = leftHindTip(0);
//            p4.x = rightHindTip(0);
//            p1.y = leftTip(1);
//            p2.y = rightTip(1);
//            p3.y = leftHindTip(1);
//            p4.y = rightHindTip(1);

//            // Attention: TODO mean position is important!!! (mean x and y..)
//            p1.z = -(sol(0) * (leftTip(0) - meanOfAllFootTips(0)) + sol(1) * (leftTip(1) - meanOfAllFootTips(1))) + meanZelevation;
//            p2.z = -(sol(0) * (rightTip(0) - meanOfAllFootTips(0)) + sol(1) * (rightTip(1) - meanOfAllFootTips(1))) + meanZelevation;
//            p3.z = -(sol(0) * (leftHindTip(0) - meanOfAllFootTips(0)) + sol(1) * (leftHindTip(1) - meanOfAllFootTips(1))) + meanZelevation;
//            p4.z = -(sol(0) * (rightHindTip(0) - meanOfAllFootTips(0)) + sol(1) * (rightHindTip(1) - meanOfAllFootTips(1))) + meanZelevation;
//            footTipPlaneFitVisualization_.points.push_back(p1);
//            footTipPlaneFitVisualization_.points.push_back(p2);
//            footTipPlaneFitVisualization_.points.push_back(p3);
//            footTipPlaneFitVisualization_.points.push_back(p4);

//            // DUDUDUDUDU
//            planeFitVisualizationPublisher_.publish(footTipPlaneFitVisualization_);

//            // Calculate squared difference from plane:
//            double variancePlaneFit = pow(p1.z - leftTip(2), 2) + pow(p2.z - rightTip(2), 2) +
//                    pow(p3.z - leftHindTip(2), 2) + pow(p4.z - rightHindTip(2), 2);
//      //      std::cout << "variancePlaneFit: " << variancePlaneFit << std::endl;

    geometry_msgs::Twist varianceMsg;
    varianceMsg.linear.x = varianceConsecutiveFootTipPositions;
    varianceMsg.linear.z = varianceConsecutiveFootTipPositions;// + variancePlaneFit;

    // set function..
    setTerrainVariance(varianceMsg.linear.z);

    //varianceMsg.angular.x = getPenetrationDepthVariance();
           // footTipPlaneFitVisualization_.action = visualization_msgs::Marker::;

            // TODO: Do visualize fitted Plane!!
            //varianceTwistPublisher_.publish(varianceMsg);



    // TODO: add Layer, that takes variace uncertainty into account.
}

bool SupportSurfaceEstimation::setTerrainVariance(double& terrainVariance){
    terrainVariance_ = terrainVariance;
}

double SupportSurfaceEstimation::getTerrainVariance(){
    return terrainVariance_;
}

} /* namespace */
