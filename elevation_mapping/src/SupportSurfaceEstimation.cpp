/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/SupportSurfaceEstimation.hpp"
#include "elevation_mapping/ElevationMap.hpp"


using namespace std;
using namespace grid_map;

namespace elevation_mapping {

SupportSurfaceEstimation::SupportSurfaceEstimation(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap"),
      filterChain2_("grid_map::GridMap")
{
  // Publisher for support surface map.
  elevationMapGPPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("support_surface_gp", 1);

  // Publisher for plotting dynamic values, such as terrain variance, characteristic value and sinkage depth variance.
  varianceTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("variances", 1000);

  // Publisher for visualization of tile centers for gaussian process regression.
  supportSurfaceAddingAreaPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("adding_area", 1000);

  // Set the constructor parameters.
  setParameters();
}

SupportSurfaceEstimation::~SupportSurfaceEstimation()
{
}

void SupportSurfaceEstimation::setParameters(){

    // Config Parameters.
    nodeHandle_.param("add_old_support_surface_data_to_gp_training", addOldSupportSurfaceDataToGPTraining_, false);
    nodeHandle_.param("weight_terrain_continuity", weightTerrainContinuity_, 1.0);
    nodeHandle_.param("run_terrain_continuity_biasing", runTerrainContinuityBiasing_, true);
    nodeHandle_.param("exponent_sinkage_depth_weight", exponentSinkageDepthWeight_, 2.0);
    nodeHandle_.param("exponent_terrain_continuity_weight", exponentTerrainContinuityWeight_, 2.0);
    nodeHandle_.param("weight_decay_threshold", weightDecayThreshold_, 0.4);
    nodeHandle_.param("exponent_characteristic_value", exponentCharacteristicValue_, 1.0);
    nodeHandle_.param("continuity_filter_gain", continuityFilterGain_, 0.3);
    nodeHandle_.param("gp_lengthscale", GPLengthscale_, 10.0);
    nodeHandle_.param("gp_sigma_n", GPSigmaN_, 0.1);
    nodeHandle_.param("gp_sigma_f", GPSigmaF_, 0.001);
    nodeHandle_.param("tile_resolution", tileResolution_, 0.08);
    nodeHandle_.param("tile_diameter", tileDiameter_, 0.23);
    nodeHandle_.param("side_length_adding_patch", sideLengthAddingPatch_, 1.3);
    nodeHandle_.param("use_bag", useBag_, false);
    nodeHandle_.param("run_hind_leg_support_surface_estimation", runHindLegSupportSurfaceEstimation_, false);
    nodeHandle_.param("continuity_gp_lengthscale", continuityGPLengthscale_, 10.0);
    nodeHandle_.param("continuity_gp_sigma_n", continuityGPSigmaN_, 0.1);
    nodeHandle_.param("continuity_gp_sigma_f", continuityGPSigmaF_, 0.001);
    nodeHandle_.param("continuity_gp_nn_lengthscale", continuityGPNNLengthscale_, 10.0);
    nodeHandle_.param("continuity_gp_nn_sigma_n", continuityGPNNSigmaN_, 0.1);
    nodeHandle_.param("continuity_gp_nn_sigma_f", continuityGPNNSigmaF_, 0.001);
    nodeHandle_.param("continuity_gp_nn_beta", continuityGPNNBeta_, 0.001);

    nodeHandle_.param("continuity_gp_c_c", continuityGPCC_, 10.0);
    nodeHandle_.param("continuity_gp_kernel", continuityGPKernel_, std::string("nn"));

    nodeHandle_.param("continuity_gp_hy_a", continuityHYa_, 10.0);
    nodeHandle_.param("continuity_gp_hy_b", continuityHYb_, 10.0);
    nodeHandle_.param("continuity_gp_hy_sigma_f", continuityGPHYSigmaF_, 10.0);
    nodeHandle_.param("continuity_gp_hy_sigma_n", continuityGPHYSigmaN_, 10.0);

    nodeHandle_.param("sinkage_gp_lengthscale", sinkageGPLengthscale_, 0.1);
    nodeHandle_.param("sinkage_gp_sigma_n", sinkageGPSigmaN_, 0.1);
    nodeHandle_.param("sinkage_gp_sigma_f", sinkageGPSigmaF_, 0.001);
    nodeHandle_.param("sinkage_gp_kernel", sinkageGPKernel_, std::string("ou"));

    nodeHandle_.param("continuity_gp_ousqe_lengthscale", continuityGPOUSQELengthscale_, 0.6);

    // rq kernel params
    nodeHandle_.param("continuity_gp_rq_lengthscale", continuityGPRQLengthscale_, 0.5);
    nodeHandle_.param("continuity_gp_rq_sigma_f", continuityGPRQSigmaF_, 0.5);
    nodeHandle_.param("continuity_gp_rq_sigma_n", continuityGPRQSigmaN_, 0.5);
    nodeHandle_.param("continuity_gp_rq_alpha", continuityGPRQa_, 0.5);

    // Ou continuity gp params.
    nodeHandle_.param("continuity_gp_ou_lengthscale", continuityGPOULengthscale_, 0.1);
    nodeHandle_.param("continuity_gp_ou_sigma_n", continuityGPOUSigmaN_, 0.1);
    nodeHandle_.param("continuity_gp_ou_sigma_f", continuityGPOUSigmaF_, 0.001);

    nodeHandle_.param("use_sign_selective_continuity_filter", useSignSelectiveContinuityFilter_, true);
    nodeHandle_.param("sign_selective_continuity_filter_gain", signSelectiveContinuityFilterGain_, 0.5);
    nodeHandle_.param("sinkage_depth_filter_gain_up", sinkageDepthFilterGainUp_, 0.1);
    nodeHandle_.param("sinkage_depth_filter_gain_down", sinkageDepthFilterGainDown_, 0.1);

    nodeHandle_.param("run_drift_refinement_support_surface", runDriftRefinementSupportSurface_, false);
    nodeHandle_.param("add_estimated_sinkage_depth_data_ahead", addEstimatedSinkageDepthDataAhead_, false);

    nodeHandle_.param("size_support_surface_update", sizeSupportSurfaceUpdate_, std::string("large"));

    nodeHandle_.param("averaging_version", averagingVersion_, false);

    nodeHandle_.param("continuity_variance_factor", continuityVarianceFactor_, 0.15);

    nodeHandle_.param("y_constraint_for_visualization", yConstraintForVisualization_, false);

    // Initializations.
    supportSurfaceInitializationTrigger_ = false;
    cumulativeSupportSurfaceUncertaintyEstimation_ = 0.0;
    leftFrontSinkageDepth_ = 0.0;
    rightFrontSinkageDepth_ = 0.0;
    leftHindSinkageDepth_ = 0.0;
    rightHindSinkageDepth_ = 0.0;
    lowPassFilteredSinkageDepthVariance_ = 0.0;
    terrainVarianceFront_ = 0.0;
    terrainVarianceHind_ = 0.0;
    initializedLeftSinkageDepth_ = initializedRightSinkageDepth_ = initializedLeftHindSinkageDepth_ = initializedRightHindSinkageDepth_ = false;

    totalMeanDuration_ = 0.0;
    meanDurationCounter_ = 0;

    mainGPTotalDuration_ = 0.0;
    mainGPDurationCounter_ = 0;

    supportSurfaceUncertaintyEstimationCounter_ = 0;

    // Initializations for ground truth comparison in simulation.
    if (!useBag_) {
        overallConsideredStanceCounter_ = 0;
        overallSummedMeanElevationDifference_ = 0.0;
        overallMeanElevationDifference_ = 0.0;
    }

    // Visualization in Rviz (centers of regression tiles).
    tileMarkerList_.header.frame_id = "odom_drift_adjusted";
    tileMarkerList_.header.stamp = ros::Time();
    tileMarkerList_.ns = "elevation_mapping";
    tileMarkerList_.id = 0;
    tileMarkerList_.type = visualization_msgs::Marker::SPHERE_LIST;
    tileMarkerList_.action = visualization_msgs::Marker::ADD;
    tileMarkerList_.pose.orientation.x = 0.0;
    tileMarkerList_.pose.orientation.y = 0.0;
    tileMarkerList_.pose.orientation.z = 0.0;
    tileMarkerList_.pose.orientation.w = 1.0;
    tileMarkerList_.scale.x = 0.1;
    tileMarkerList_.scale.y = 0.1;
    tileMarkerList_.scale.z = 0.1;
    tileMarkerList_.color.a = 1.0;
    tileMarkerList_.color.r = 1.0;
    tileMarkerList_.color.g = 0.1;
    tileMarkerList_.color.b = 0.1;
}

bool SupportSurfaceEstimation::updateSupportSurfaceEstimation(std::string tip, GridMap& rawMap,
                                                              GridMap& supportMap, GridMap& fusedMap, Eigen::Vector3f& stance, const double totalEstimatedDrift){
    // Check if foot tip is inside the elevation map.
    Position tipPosition(stance(0), stance(1));
    if(rawMap.isInside(tipPosition)) {

    // Proprioceptive terrain variance estimation.
    proprioceptiveRoughnessEstimation(tip, stance);

        if (tip != "lefthind" && tip != "righthind" || runHindLegSupportSurfaceEstimation_
                && leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0) {

            // Run only if stance vectors are initialized..
            if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0) {

                //simpleTerrainContinuityLayer(tip, supportMap);
                terrainContinuityLayerGP(tip, supportMap, totalEstimatedDrift);

                //terrainContinuityLayerGPwhole(tip, supportMap);

                // Smoothed Top Layer to get vertical Difference between top layer and foot tip position (sinkage depth)
                setSmoothedTopLayer(tip, rawMap, supportMap);
                double verticalDifference = getFootTipElevationMapDifferenceGP(tip, supportMap);

                sinkageDepthVarianceEstimation(tip, verticalDifference);
                //! Yes all this!

                // Uncertainty Estimation based on low pass filtered error between foot tip height and predicted support Surface.
                //if (tip == "left" || tip == "right") setSupportSurfaceUncertaintyEstimation(tip, supportMap); //! TODO: uncomment whats inside of this function.
                setSupportSurfaceUncertaintyEstimation(tip, supportMap, totalEstimatedDrift);

                // If choosing "large" make the size of the GP larger.
                if (sizeSupportSurfaceUpdate_ == "large") sideLengthAddingPatch_ = 1.37;

                // Main gaussian process regression.
                mainGPRegression(tileResolution_, tileDiameter_, sideLengthAddingPatch_,
                                 tip, verticalDifference, rawMap, supportMap, fusedMap);
            }
        }

        // Ground truth difference evaluation for simulation environment.
        if (!useBag_) testTrackMeanSupportErrorEvaluation(supportMap);
    }
    else ROS_WARN("Foot tip considered not to be inside the valid area of the elevation map. \n");

    // Set the raw Elevation Map layer to be the upper bound of the support surface estimation.
    //supportSurfaceUpperBoundingGP(rawMap, supportMap);

    // Publish map
    grid_map_msgs::GridMap mapMessageGP;
    GridMapRosConverter::toMessage(supportMap, mapMessageGP);
    mapMessageGP.info.header.frame_id = "odom_drift_adjusted";
    elevationMapGPPublisher_.publish(mapMessageGP);
    return true;
}

Position3 SupportSurfaceEstimation::getFootTipPosition3(std::string tip){
    Position3 tipPos;
    if (tip == "left") tipPos = {frontLeftFootTip_(0), frontLeftFootTip_(1), frontLeftFootTip_(2)};
    if (tip == "right") tipPos = {frontRightFootTip_(0), frontRightFootTip_(1), frontRightFootTip_(2)};
    if (tip == "lefthind") tipPos = {hindLeftFootTip_(0), hindLeftFootTip_(1), hindLeftFootTip_(2)};
    if (tip == "righthind") tipPos = {hindRightFootTip_(0), hindRightFootTip_(1), hindRightFootTip_(2)};
    return tipPos;
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

bool SupportSurfaceEstimation::setSupportSurfaceUncertaintyEstimation(std::string tip, GridMap& supportMap, const double & totalEstimatedDrift){

    Position3 latestTip = getFootTipPosition3(tip);
    // So far not low pass filtered!! -> create Vector for that (TODO)
    grid_map::Position latestTipPosition(latestTip(0), latestTip(1));

    // As support Map lives in the corrected frame.
    double diff = fabs((latestTip(2) - totalEstimatedDrift) - supportMap.atPosition("elevation", latestTipPosition));

    if (!isnan(diff)) {
        supportSurfaceUncertaintyEstimationCounter_++;
        supportSurfaceUncertaintyEstimation_ = diff;
        cumulativeSupportSurfaceUncertaintyEstimation_ += diff;
    }
    double meanFootTipPlacementUncertainty = cumulativeSupportSurfaceUncertaintyEstimation_ / supportSurfaceUncertaintyEstimationCounter_;
    std::cout << "### +++ Mean SUPPORT SURFACE UNCERTAINTY ESTIMATION: " << meanFootTipPlacementUncertainty << "   tip:::: " << tip << std::endl;
    return true;
}

double SupportSurfaceEstimation::getFootTipElevationMapDifferenceGP(std::string tip, GridMap& supportMap){

    double radius = 0.7;
    Position3 footTip3 = getFootTipPosition3(tip);
    Position footTipHorizontal(footTip3(0), footTip3(1));
    double verticalDifference;
    if (supportMap.exists("smoothed_top_layer_gp")) {
        if(supportMap.isInside(footTipHorizontal)){
            if (!isnan(supportMap.atPosition("smoothed_top_layer_gp", footTipHorizontal))){
                verticalDifference = footTip3(2) - supportMap.atPosition("smoothed_top_layer_gp", footTipHorizontal); // Hacked to rawMap
            }
            else verticalDifference = getClosestMapValueUsingSpiralIteratorElevation(supportMap, footTipHorizontal, radius, footTip3(2)); // New experiment.. Wrong, not difference yet!!!
        }
    }
    else verticalDifference = std::numeric_limits<double>::quiet_NaN();
   // std::cout << "Vertical Difference!! -> ->: " << verticalDifference << std::endl;
    return verticalDifference;
}

double SupportSurfaceEstimation::getClosestMapValueUsingSpiralIteratorElevation(GridMap& MapReference, Position footTip, double radius, double tipHeight){

    int counter = 0;
    for (SpiralIterator iterator(MapReference, footTip, radius);
         !iterator.isPastEnd(); ++iterator) {   // Hacked to is inside..
        Index index(*iterator);
        Position pos;
        MapReference.getPosition(index, pos);
        if (MapReference.isInside(pos) && !isnan(MapReference.at("smoothed_top_layer_gp", index)) && MapReference.isValid(index)){
          //  std::cout << "RETURNED DIFFERENCE TO A CLOSE NEIGHBOR USING SPIRALLING!!!" << std::endl;
            return tipHeight - MapReference.at("smoothed_top_layer_gp", index); // && MapReference.isValid(index)
        }
        counter++;
        if (counter > 11) break;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool SupportSurfaceEstimation::sinkageDepthVarianceEstimation(std::string tip, double verticalDifference){

    // Parameter, number of considered sinkage depth values.
    int maxHistory = 7; // Hacking!!

    if (tip == "left" || tip == "right") {

        // New, for weighting the sinkage depth variance by horizontal translation
        double horDiff;

        if (leftStanceVector_.size() > 1 && tip == "left"){
            horDiff = double(sqrt(pow(leftStanceVector_[1](0) - leftStanceVector_[0](0),2.0)+pow(leftStanceVector_[1](1) - leftStanceVector_[0](1),2.0)));
        }
        else if(rightStanceVector_.size() > 1 && tip == "right"){
            horDiff = double(sqrt(pow(rightStanceVector_[1](0) - rightStanceVector_[0](0),2.0)+pow(rightStanceVector_[1](1) - rightStanceVector_[0](1),2.0)));
        }

        // Initializations.
        double totalVerticalDifference = 0.0;
        double squaredTotalVerticalDifference = 0.0;
        double totalVerticalDifferenceChange = 0.0;
        double squaredTotalVerticalDifferenceChange = 0.0;

        // Populate Vector of vertical Difference Values.
        if (!isnan(verticalDifference)) verticalDifferenceVector_.push_back(verticalDifference); // Just Hacked in, TODO, check what it does!!
        if (verticalDifferenceVector_.size() > maxHistory) verticalDifferenceVector_.erase(verticalDifferenceVector_.begin());

        // Get the number of sinkage depth values considered.
        int count = verticalDifferenceVector_.size();

        // Iterate.
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

        // Variance Calculation.
        double penetrationDepthVariance = squaredTotalVerticalDifference / double(count) -
                pow(totalVerticalDifference / double(count), 2);

        // Differential Version.
        double differentialPenetrationDepthVariance = squaredTotalVerticalDifferenceChange / double(count) -
                pow(totalVerticalDifferenceChange / double(count), 2);

        // Sign selective low pass filter.
        if (!isnan(penetrationDepthVariance)) {
            if (!isnan(lowPassFilteredSinkageDepthVariance_)) {
                lowPassFilteredSinkageDepthVariance_ = signSelectiveLowPassFilter(lowPassFilteredSinkageDepthVariance_,
                                                                                  penetrationDepthVariance, sinkageDepthFilterGainDown_, sinkageDepthFilterGainUp_);
                setPenetrationDepthVariance(lowPassFilteredSinkageDepthVariance_, tip);
            }
            else {
                setPenetrationDepthVariance(penetrationDepthVariance, tip);
            }
        }
        //setDifferentialPenetrationDepthVariance(differentialPenetrationDepthVariance);
    }
    if (tip == "lefthind" || tip == "righthind") {

        double horDiff;
        if (leftHindStanceVector_.size() > 1 && tip == "lefthind"){
            horDiff = double(sqrt(pow(leftHindStanceVector_[1](0) - leftHindStanceVector_[0](0), 2.0)+pow(leftHindStanceVector_[1](1) - leftHindStanceVector_[0](1), 2.0)));
        }
        else if(rightHindStanceVector_.size() > 1 && tip == "righthind"){
            horDiff = double(sqrt(pow(rightHindStanceVector_[1](0) - rightHindStanceVector_[0](0), 2.0)+pow(rightHindStanceVector_[1](1) - rightHindStanceVector_[0](1), 2.0)));
        }

        // Initializations.
        double totalVerticalDifference = 0.0;
        double squaredTotalVerticalDifference = 0.0;
        double totalVerticalDifferenceChange = 0.0;
        double squaredTotalVerticalDifferenceChange = 0.0;

        // Populate Vector of vertical Difference Values.
        if (!isnan(verticalDifference)) verticalDifferenceVectorHind_.push_back(verticalDifference); // Just Hacked in!!
        if (verticalDifferenceVectorHind_.size() > maxHistory) verticalDifferenceVectorHind_.erase(verticalDifferenceVectorHind_.begin());

        // Get the number of sinkage depth values considered.
        int count = verticalDifferenceVectorHind_.size();

        // Iterate.
        for (auto& n : verticalDifferenceVectorHind_) {
            totalVerticalDifference += n;
            squaredTotalVerticalDifference += pow(n, 2);
        }

        // Differencial version.
        if (verticalDifferenceVectorHind_.size() > 1) {
            for (unsigned int j = 1; j < verticalDifferenceVectorHind_.size(); ++j) {
                totalVerticalDifferenceChange += verticalDifferenceVectorHind_[j] - verticalDifferenceVectorHind_[j-1];
                squaredTotalVerticalDifferenceChange += pow(verticalDifferenceVectorHind_[j] - verticalDifferenceVectorHind_[j-1], 2);
            }
        }

        // Variance Calculation.
        double penetrationDepthVariance = squaredTotalVerticalDifference / double(count) -
                pow(totalVerticalDifference / double(count), 2);

     //   std::cout << " HINDHINDHIND: " << penetrationDepthVariance << std::endl;

        // Differential Version.
        double differentialPenetrationDepthVariance = squaredTotalVerticalDifferenceChange / double(count) -
                pow(totalVerticalDifferenceChange / double(count), 2);

        // Sign selective low pass filter.
        if (!isnan(penetrationDepthVariance)) {
            if (!isnan(lowPassFilteredHindSinkageDepthVariance_)) {
                lowPassFilteredHindSinkageDepthVariance_ = signSelectiveLowPassFilter(lowPassFilteredHindSinkageDepthVariance_,
                                                                                  penetrationDepthVariance, sinkageDepthFilterGainDown_, sinkageDepthFilterGainUp_);
                setPenetrationDepthVariance(lowPassFilteredHindSinkageDepthVariance_, tip);
            }
            else {
                setPenetrationDepthVariance(penetrationDepthVariance, tip);
            }
        }
            //setDifferentialPenetrationDepthVariance(differentialPenetrationDepthVariance);
    }
    return true;
}

// For using it in the penetration depth estimation.
void SupportSurfaceEstimation::setPenetrationDepthVariance(double penetrationDepthVariance, std::string tip){
    if (tip == "left" || tip == "right")
        if (!isnan(penetrationDepthVariance)) penetrationDepthVariance_ = penetrationDepthVariance;
    if (tip == "lefthind" || tip == "righthind") {
        if (!isnan(penetrationDepthVariance)) {
         //   std::cout << "SET THE HIND PEN DEPTH VARIANCE ESTIMATION!!!!!!: \n \n \n \n \n " << penetrationDepthVariance << std::endl;
            penetrationDepthVarianceHind_ = penetrationDepthVariance;
        }
    }
}

// For using it in the penetration depth estimation.
void SupportSurfaceEstimation::setDifferentialPenetrationDepthVariance(double differentialPenetrationDepthVariance){
    if (!isnan(differentialPenetrationDepthVariance)) differentialPenetrationDepthVariance_ = differentialPenetrationDepthVariance;
}


bool SupportSurfaceEstimation::setSmoothedTopLayer(std::string tip, GridMap& rawMap, GridMap& supportMap){

    // Smoothing radius of area considered for sinkage depth calculation.
    double smoothingRadius = 0.09;

    // Gaussian Process Regression Parameters.
    int inputDim = 2;
    int outputDim = 1;
    GaussianProcessRegression<float> smoothedTopLayerGPR(inputDim, outputDim);
    smoothedTopLayerGPR.SetHyperParams(0.2, 0.1, 0.0005);

    // Get the foot tip position.
    Position3 tipPos3 = getFootTipPosition3(tip);
    Position footTipPosition(tipPos3(0), tipPos3(1));

    // Iterate around the foot tip position to add Data.
    for (CircleIterator iterator(supportMap, footTipPosition, smoothingRadius); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);
        Position inputPosition;
        supportMap.getPosition(index, inputPosition);
        trainInput(0) = inputPosition(0);
        trainInput(1) = inputPosition(1);
        trainOutput(0) = rawMap.at("elevation", index);
        if (!isnan(trainOutput(0)) && rawMap.isValid(index)){
            smoothedTopLayerGPR.AddTrainingData(trainInput, trainOutput);
        }
    }

    // Iterate around the foot tip position to get the regression value.
    if (smoothedTopLayerGPR.get_n_data() > 0){
        for (CircleIterator iterator(supportMap, footTipPosition, smoothingRadius); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            Eigen::VectorXf testInput(inputDim);
            Position testPosition;
            supportMap.getPosition(index, testPosition);
            testInput(0) = testPosition(0);
            testInput(1) = testPosition(1);
            double outputHeight = smoothedTopLayerGPR.DoRegression(testInput)(0);
            if (!isnan(outputHeight)){
                supportMap.at("smoothed_top_layer_gp", index) = outputHeight;
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

bool SupportSurfaceEstimation::proprioceptiveRoughnessEstimation(std::string tip, Eigen::Vector3f meanStance){

    //! TODO: make function: update stance vectors.
    if (tip == "left"){
        leftStanceVector_.push_back(meanStance);
        if (leftStanceVector_.size() > 2) leftStanceVector_.erase(leftStanceVector_.begin());
    }
    if (tip == "right"){
        rightStanceVector_.push_back(meanStance);
        if (rightStanceVector_.size() > 2) rightStanceVector_.erase(rightStanceVector_.begin());
    }
    if (tip == "lefthind"){
        leftHindStanceVector_.push_back(meanStance);
        if (leftHindStanceVector_.size() > 2) leftHindStanceVector_.erase(leftHindStanceVector_.begin());
    }
    if (tip == "righthind"){
        rightHindStanceVector_.push_back(meanStance);
        if (rightHindStanceVector_.size() > 2) rightHindStanceVector_.erase(rightHindStanceVector_.begin());
    }

    // Save the mean tip positions for simple foot tip embedding into high grass detection
    //! TODO: merge these two into one, unnecessary..
    if (tip == "left") frontLeftFootTip_ = meanStance;
    if (tip == "right") frontRightFootTip_ = meanStance;
    if (tip == "lefthind") hindLeftFootTip_ = meanStance;
    if (tip == "righthind") hindRightFootTip_ = meanStance;

    bool writeHorizontalFootTipEstimationStatisticsToFile = false;
    if(writeHorizontalFootTipEstimationStatisticsToFile){
        if (leftStanceVector_.size() > 1 && tip == "left"){
            double diff = double(leftStanceVector_[1](2)-leftStanceVector_[0](2));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
        else if(rightStanceVector_.size() > 1 && tip == "right"){
            double diff = double(rightStanceVector_[1](2)-rightStanceVector_[0](2));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
        else if(leftHindStanceVector_.size() > 1 && tip == "lefthind"){
            double diff = double(leftHindStanceVector_[1](2)-leftHindStanceVector_[0](2));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
        else if(rightHindStanceVector_.size() > 1 && tip == "righthind"){
            double diff = double(rightHindStanceVector_[1](2)-rightHindStanceVector_[0](2));
            //writeFootTipStatisticsToFile(diff, "/home/timon/driftEst.txt");
        }
    }
    proprioceptiveVariance(tip);
}

bool SupportSurfaceEstimation::proprioceptiveVariance(std::string tip){

    double diff = 0.0;

    // New to weigh the

    double meanDrift = 0.467;
    if (leftStanceVector_.size() > 1 && tip == "left"){
        diff = double((leftStanceVector_[1](2) - leftStanceVector_[0](2)));
    }
    else if(rightStanceVector_.size() > 1 && tip == "right"){
        diff = double((rightStanceVector_[1](2) - rightStanceVector_[0](2)));
    }
    else if(leftHindStanceVector_.size() > 1 && tip == "lefthind"){
        diff = double((leftHindStanceVector_[1](2) - leftHindStanceVector_[0](2)));
    }
    else if(rightHindStanceVector_.size() > 1 && tip == "righthind"){
        diff = double((rightHindStanceVector_[1](2) - rightHindStanceVector_[0](2)));
    }

    if (tip == "left" || tip == "right") proprioceptiveDiffVectorFront_.push_back(diff);
    if (proprioceptiveDiffVectorFront_.size() > 7) proprioceptiveDiffVectorFront_.erase(proprioceptiveDiffVectorFront_.begin());
    if (tip == "lefthind" || tip == "righthind") proprioceptiveDiffVectorHind_.push_back(diff);
    if (proprioceptiveDiffVectorHind_.size() > 7) proprioceptiveDiffVectorHind_.erase(proprioceptiveDiffVectorHind_.begin());

    //proprioceptiveDiffVector_.push_back(diff);
    //if (proprioceptiveDiffVector_.size() > 12) proprioceptiveDiffVector_.erase(proprioceptiveDiffVector_.begin());


    double varianceConsecutiveFootTipPositions;

    if (tip == "left" || tip == "right") {

        // Calculate Variance.
        double total = 0.0;
        double totalSquared = 0.0;
        for (auto& n : proprioceptiveDiffVectorFront_){
            total += n;
            totalSquared += pow(n, 2);
        }
        varianceConsecutiveFootTipPositions = totalSquared / (double)proprioceptiveDiffVectorFront_.size() - pow(total/(double)proprioceptiveDiffVectorFront_.size(), 2);
        //double mean = total/(double)varianceDiffVector.size();
    }
    if (tip == "lefthind" || tip == "righthind") {
        if (runHindLegSupportSurfaceEstimation_){
            // Calculate Variance.
            double total = 0.0;
            double totalSquared = 0.0;
            for (auto& n : proprioceptiveDiffVectorHind_){
                total += n;
                totalSquared += pow(n, 2);
            }
            varianceConsecutiveFootTipPositions = totalSquared / (double)proprioceptiveDiffVectorHind_.size() - pow(total/(double)proprioceptiveDiffVectorHind_.size(), 2);
            //double mean = total/(double)varianceDiffVector.size();
        }
    }

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
    setTerrainVariance(varianceConsecutiveFootTipPositions, tip);

    //varianceMsg.angular.x = getPenetrationDepthVariance();
           // footTipPlaneFitVisualization_.action = visualization_msgs::Marker::;

            // TODO: Do visualize fitted Plane!!
            //varianceTwistPublisher_.publish(varianceMsg);



    // TODO: add Layer, that takes variace uncertainty into account.
}

bool SupportSurfaceEstimation::setTerrainVariance(double& terrainVariance, std::string tip){
    if (tip == "left" || tip == "right")
        terrainVarianceFront_ = terrainVariance;
    if (runHindLegSupportSurfaceEstimation_ && (tip == "lefthind" || tip == "righthind"))
        terrainVarianceHind_ = terrainVariance;
}

double SupportSurfaceEstimation::getTerrainVariance(std::string tips){
    if (tips == "front")
        return terrainVarianceFront_;
    if (runHindLegSupportSurfaceEstimation_ && tips == "hind")
        return terrainVarianceHind_;
}

double SupportSurfaceEstimation::getPenetrationDepthVariance(std::string tips){
    if (tips == "front") return penetrationDepthVariance_;
    if (tips == "hind") return penetrationDepthVarianceHind_;
}

double SupportSurfaceEstimation::getDifferentialPenetrationDepthVariance(){
    return differentialPenetrationDepthVariance_;
}

bool SupportSurfaceEstimation::mainGPRegression(double tileResolution, double tileDiameter, double sideLengthAddingPatch, std::string tip,
                                                const double tipDifference, GridMap& rawMap, GridMap& supportMap, GridMap& fusedMap){


    double directOutput;


    // Set start time for time calculation for main GP regression.
    const ros::WallTime mainGPStartTime(ros::WallTime::now());

    if (2.0 * tileResolution > tileDiameter) { // TODO: make this double, as using circle iterator
        std::cout << "tile size for gaussian process model tiling must be higher than twice the tile Resolution" << std::endl;
        return false;
    }


    const ros::WallDuration duration21 = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN:::::::::iterated through all the tiles in %f s.", duration21.toSec());

    // Update sinkage depth map.
    simpleSinkageDepthLayer(tip, tipDifference, supportMap, rawMap);
    //sinkageDepthLayerGP(tip, tipDifference, supportMap);

    const ros::WallDuration duration22 = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN:::::::::iterated through all the tiles in %f s.", duration22.toSec());

    // Tests..
    meanDurationCounter_++;
    totalMeanDuration_ += (duration22.toSec() - duration21.toSec());
    std::cout << "local now duration: " << (duration22.toSec() - duration21.toSec()) << std::endl;
    std::cout << "meanTotalDuration: ######################::::    #######:  "
              << totalMeanDuration_ / (double)meanDurationCounter_ << std::endl;


    // Subparams of tiling.
    int noOfTilesPerHalfSide = floor((sideLengthAddingPatch / 2.0) / tileResolution);
    double radius = (sideLengthAddingPatch - 0.1) / 2.0;

    // Get uncertainty measures.
    double terrainVariance = getTerrainVariance("front");
    double terrainVarianceHind = getTerrainVariance("hind");


    double supportSurfaceUncertaintyEstimation = getSupportSurfaceUncertaintyEstimation();
    double sinkageVariance = getPenetrationDepthVariance("front");
    double sinkageVarianceHind = getPenetrationDepthVariance("hind");
    //double differentialSinkageVariance = getDifferentialPenetrationDepthVariance();
    //double cumulativeSupportSurfaceUncertaintyEstimation = getCumulativeSupportSurfaceUncertaintyEstimation();

    if (tip == "left" || tip == "right") {

        // Calculate characteristic Value.
        double characteristicValue = (weightTerrainContinuity_ * terrainVariance) / pow(sinkageVariance, exponentCharacteristicValue_);

        // Low pass filtering and bounding of the continuity caracteristic value.
        if (useSignSelectiveContinuityFilter_) {
            // Sign Selective low pass filter.
            if (lowPassFilteredTerrainContinuityValue_ < characteristicValue) lowPassFilteredTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                                    * characteristicValue, 0.44), 4.5); // TODO: reason about bounding.
            else lowPassFilteredTerrainContinuityValue_ = characteristicValue;
        }
        else lowPassFilteredTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                            * characteristicValue, 0.44), 4.5); // TODO: reason about bounding.
    }
    if (tip == "lefthind" || tip == "righthind") {

      //  std::cout << "terr vat hind:   -> -> -> ................................." << terrainVarianceHind << std::endl;
      //  std::cout << "sink vat hind:   -> -> -> ................................." << sinkageVarianceHind << std::endl;



        // Calculate characteristic Value.
        double characteristicValue = (weightTerrainContinuity_ * terrainVarianceHind) / pow(sinkageVarianceHind, exponentCharacteristicValue_);


        // Low pass filtering and bounding of the continuity caracteristic value.
        if (useSignSelectiveContinuityFilter_) {
            // Sign Selective low pass filter.
            if (lowPassFilteredHindTerrainContinuityValue_ < characteristicValue) lowPassFilteredHindTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredHindTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                                    * characteristicValue, 0.44), 3.5); // TODO: reason about bounding.
            else lowPassFilteredHindTerrainContinuityValue_ = characteristicValue;
        }
        else lowPassFilteredHindTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredHindTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                            * characteristicValue, 0.44), 3.5); // TODO: reason about bounding.
    }

    // Set adaptation message values.
    geometry_msgs::TwistStamped adaptationMsg;
    adaptationMsg.header.stamp = ros::Time::now();
    adaptationMsg.twist.linear.x = terrainVariance;
    adaptationMsg.twist.linear.y = sinkageVariance;
    adaptationMsg.twist.linear.z = lowPassFilteredTerrainContinuityValue_;
    adaptationMsg.twist.angular.x = terrainVarianceHind;
    adaptationMsg.twist.angular.y = sinkageVarianceHind;
    adaptationMsg.twist.angular.z = lowPassFilteredHindTerrainContinuityValue_;

    // Get the foot Tip Position.
    Position3 footTip3 = getFootTipPosition3(tip);
    Position footTip(footTip3(0), footTip3(1));



    const ros::WallDuration duration1 = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN::::::::::::: GP regression initialization was done in %f s.", duration1.toSec());


    // Iterate through the model tiles.
    for (int i = -noOfTilesPerHalfSide; i <= noOfTilesPerHalfSide; ++i){
        for (int j = -noOfTilesPerHalfSide; j <= noOfTilesPerHalfSide; ++j){
            if (sqrt(pow(i * tileResolution,2) + pow(j * tileResolution,2)) < radius){

                // GP Parameters -> to move outside of the loop and apply the data clear method (TODO!)
                int inputDim = 2;
                int outputDim = 1;
                GaussianProcessRegression<float> myGPR(inputDim, outputDim);
                myGPR.SetHyperParams(GPLengthscale_, GPSigmaN_, GPSigmaF_);

                // Get the tile position.
                Position posTile;
                posTile(0) = footTip(0) + i * tileResolution;
                posTile(1) = footTip(1) + j * tileResolution;

                // Loop to add training data to GP regression.
                for (CircleIterator iterator(supportMap, posTile, tileDiameter/2.0); !iterator.isPastEnd(); ++iterator) {
                    const Index index(*iterator);
                    Eigen::VectorXf trainInput(inputDim);
                    Eigen::VectorXf trainOutput(outputDim);

                    // Get Position of cell and set as input data for GP training.
                    Position cellPos;
                    supportMap.getPosition(index, cellPos); // This is wrong!!!!!!!!
                    trainInput(0) = cellPos(0);
                    trainInput(1) = cellPos(1);

                    // Set Difference between sinkage depth layer and elevation layer as training output for GP.
                    trainOutput(0) = rawMap.at("elevation", index) - supportMap.at("sinkage_depth_gp", index);

                    // SomeTests
                    //Eigen::VectorXf trainOutput2(outputDim);
                    //trainOutput2(0) = supportMap.at("elevation", index);

                    // Probability sampling and replace the training data by plane value
                    double samplingValue;
                    if (tip == "left" || tip == "right") samplingValue = lowPassFilteredTerrainContinuityValue_;
                    if (tip == "lefthind" || tip == "righthind") samplingValue = lowPassFilteredHindTerrainContinuityValue_;


                    bool insert = sampleContinuityPlaneToTrainingData(cellPos, footTip, samplingValue);
                    if (insert) {
                        trainOutput(0) = supportMap.at("terrain_continuity_gp", index);
                    }
                    if (averagingVersion_) {
                        double distance = sqrt(pow(cellPos(0) - footTip(0), 2) + pow(cellPos(1) - footTip(1), 2));
                        double prob = exp(-(samplingValue * distance));
                        trainOutput(0) = (rawMap.at("elevation", index) - supportMap.at("sinkage_depth_gp", index)) * (1.0 - prob) +
                                supportMap.at("terrain_continuity_gp", index) * prob;
                    }
                    if (isnan(trainOutput(0))) trainOutput(0) = supportMap.at("terrain_continuity_gp", index);

                    // Add the training data to GP regression.
                    if (!isnan(trainOutput(0))) myGPR.AddTrainingData(trainInput, trainOutput);
                    //if (!isnan(trainOutput(0))) directOutput = trainOutput(0);
                }
                //std::cout << "The Number of Data in this tile is: " << myGPR.get_n_data() << std::endl;

                // Only perform regression if no. of Data is sufficiently high.
                if (myGPR.get_n_data() > 0){

                    // Loop here to get test output
                    for (CircleIterator iterator(supportMap, posTile, tileDiameter/2.0); !iterator.isPastEnd(); ++iterator) {
                        const Index index(*iterator);
                        auto& supportMapElevationGP = supportMap.at("elevation_gp", index);
                        auto& supportMapVarianceGP = supportMap.at("variance_gp", index);

                        // Prepare test input.
                        Eigen::VectorXf testInput(inputDim);
                        Position inputPosition;
                        supportMap.getPosition(index, inputPosition);
                        testInput(0) = inputPosition(0);
                        testInput(1) = inputPosition(1);

                        // Perform regression.
                        double regressionOutput = myGPR.DoRegression(testInput)(0);
                        double regressionOutputVariance = 0.0; // * sinkage / dist

                        // Hacked away to check if it speeds things up..
                        //double regressionOutput = myGPR.DoRegressionVariance(testInput)(0);
                        //double regressionOutputVariance = fabs(myGPR.GetVariance()(0));

                        if (isnan(supportMapElevationGP)) supportMapElevationGP = regressionOutput;
                        else supportMapElevationGP = 0.5 *  supportMapElevationGP + regressionOutput * 0.5;

                        if (isnan(supportMapVarianceGP)) supportMapVarianceGP = regressionOutputVariance;
                        else supportMapVarianceGP = 0.5 * supportMapVarianceGP + regressionOutputVariance * 0.5;
                    }
                }
                // Visualization of tile centers.
                geometry_msgs::Point p;
                p.x = posTile(0);
                p.y = posTile(1);
                p.z = 0.0;
                tileMarkerList_.points.push_back(p);
                supportSurfaceAddingAreaPublisher_.publish(tileMarkerList_);
               // myGPR.ClearTrainingData();
            }
        }
    }

    const ros::WallDuration duration2 = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN:::::::::iterated through all the tiles in %f s.", duration2.toSec());

    // Adding procedure.
    for (CircleIterator iterator(supportMap, footTip, radius); !iterator.isPastEnd(); ++iterator) { // HAcked to raw map for testing..
        const Index index(*iterator);

        Position addingPosition;
        supportMap.getPosition(index, addingPosition);
        double distance = sqrt(pow(addingPosition(0) - footTip(0), 2) +
                               pow(addingPosition(1) - footTip(1), 2));

        // Old weighting scheme.
        //double distanceFactor = sqrt(pow(addingPosition(0) - footTip(0), 2) +
        //                                   pow(addingPosition(1) - footTip(1), 2)) / radius;

        // New weighting scheme here:
        double weight;
        if (distance <= 0.2 * radius) weight = 1.0 - (distance / radius);
        else if (distance >= 0.2 * radius && distance <= weightDecayThreshold_ * radius) weight = 0.8;
        else weight = weightDecayThreshold_ - weightDecayThreshold_ * (distance - weightDecayThreshold_ * radius)
                / ((1.0 - weightDecayThreshold_) * radius);
        // End of new weighting scheme

        //test :
        //weight = fabs(max(1 - (distance / radius), 0.0)); // Linear weight, differences, artefacts, frame correction in high grass?
        //weight = fabs(exp(-distance/radius));


        // Get the various layers.
        auto& supportMapElevationGP = supportMap.at("elevation_gp", index);
        auto& supportMapElevationGPAdded = supportMap.at("elevation", index);
        auto& supportMapVarianceGP = supportMap.at("variance_gp", index);
        auto& supportMapVarianceGPAdded = supportMap.at("variance", index);



        // Added yConstraintVersion for orthonormal visualization in gazebo world.
        if (yConstraintForVisualization_){
            if (addingPosition(1) < 0.015 && addingPosition(1) > -0.015) {
                if (!isnan(supportMapElevationGPAdded)) {
                    supportMapElevationGPAdded = (1 - weight) * supportMapElevationGPAdded + supportMapElevationGP * weight;
                }
                else {
                    supportMapElevationGPAdded = supportMapElevationGP;
                }
            }
        }
        else {
            if (!isnan(supportMapElevationGPAdded)) {
                supportMapElevationGPAdded = (1 - weight) * supportMapElevationGPAdded + supportMapElevationGP * weight;
            }
            else {
                supportMapElevationGPAdded = supportMapElevationGP;
            }
        }

//        if (!isnan(supportMapElevationGPAdded)) {
//            supportMapElevationGPAdded = (1 - weight) * supportMapElevationGPAdded + directOutput * weight;
//        }
//        else {
//            supportMapElevationGPAdded = directOutput;
//        }

        double distanceVarianceExponent = 1.0;
        double heightVarianceExponent = 0.2;

        if (!isnan(supportMapVarianceGPAdded)) {
            //supportMapVarianceGPAdded = (1 - weight) * supportMapVarianceGPAdded + supportMapVarianceGP * weight; // test!!!
            if (!isnan(supportMap.at("terrain_continuity_variance_gp", index) + supportMap.at("sinkage_depth_variance_gp", index)))
                supportMapVarianceGPAdded = (1 - weight) * supportMapVarianceGPAdded +
                        (supportMap.at("terrain_continuity_variance_gp", index) + 0.0 * supportMap.at("sinkage_depth_variance_gp", index)) * weight;
        }
        else {
            supportMapVarianceGPAdded = supportMap.at("terrain_continuity_variance_gp", index) + 0.0 *
                    supportMap.at("sinkage_depth_variance_gp", index);
        }

        // New variance scheme..
        //supportMapVarianceGPAdded = supportMapVarianceGPAdded * distance * supportMap.at("sinkage_depth_gp", index) * 10000.0;


//        if (!useBag_){
//            auto& fusedMapElevationGP = fusedMap.at("elevation_gp_added_raw", index);
//            if (!isnan(supportMapElevationGPAdded)) fusedMapElevationGP = supportMapElevationGPAdded;
//        }
    }


    // New, to enlarge the area covered by the data..
    if (addEstimatedSinkageDepthDataAhead_) {
        for (GridMapIterator iterator(rawMap); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            if (isnan(supportMap.at("elevation", index))) {
                auto& sinkageDepth = supportMap.at("sinkage_depth_gp", index);
                if (!isnan(sinkageDepth)) {
                    supportMap.at("elevation", index) = rawMap.at("elevation", index) - sinkageDepth;
                }
            }
        }
    }

    const ros::WallDuration duration = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN:::::::::::: GP regression was run totally, TOTALLYTOTALLY::::::::::: in %f s.", duration.toSec());

    mainGPDurationCounter_++;
    mainGPTotalDuration_ += duration.toSec();
    std::cout << "THE OVERALL MEAN TIME FOR MAIN GP REGRESSION: " << mainGPTotalDuration_ / double(mainGPDurationCounter_) << std::endl;

    // Publish adaptation Parameters
    varianceTwistPublisher_.publish(adaptationMsg);
    return true;
}


bool SupportSurfaceEstimation::mainGPRegressionNoTiles(double tileResolution, double tileDiameter, double sideLengthAddingPatch, std::string tip,
                                                const double tipDifference, GridMap& rawMap, GridMap& supportMap, GridMap& fusedMap){


    // Set start time for time calculation for main GP regression.
    const ros::WallTime mainGPStartTime(ros::WallTime::now());

    if (2.0 * tileResolution > tileDiameter) { // TODO: make this double, as using circle iterator
        std::cout << "tile size for gaussian process model tiling must be higher than twice the tile Resolution" << std::endl;
        return false;
    }

    // Update sinkage depth map.
    simpleSinkageDepthLayer(tip, tipDifference, supportMap, rawMap);

    // Subparams of tiling.
    int noOfTilesPerHalfSide = floor((sideLengthAddingPatch / 2.0) / tileResolution);
    double radius = (sideLengthAddingPatch - 0.15) / 2.0;

    // Get uncertainty measures.
    double terrainVariance = getTerrainVariance(tip);
    double supportSurfaceUncertaintyEstimation = getSupportSurfaceUncertaintyEstimation();
    double sinkageVariance = getPenetrationDepthVariance(tip);
    //double differentialSinkageVariance = getDifferentialPenetrationDepthVariance();
    //double cumulativeSupportSurfaceUncertaintyEstimation = getCumulativeSupportSurfaceUncertaintyEstimation();

    // Calculate characteristic Value.
    double characteristicValue = (weightTerrainContinuity_ * terrainVariance) / pow((1.0 * sinkageVariance), exponentCharacteristicValue_);

    // Low pass filtering and bounding of the continuity caracteristic value.
    if (useSignSelectiveContinuityFilter_) {
        // Sign Selective low pass filter.
        if (lowPassFilteredTerrainContinuityValue_ < characteristicValue) lowPassFilteredTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                                * characteristicValue, 0.44), 4.4); // TODO: reason about bounding.
        else lowPassFilteredTerrainContinuityValue_ = characteristicValue;
    }
    else lowPassFilteredTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                        * characteristicValue, 0.44), 4.4); // TODO: reason about bounding.


    // Set message values.
    // RQT message publisher.
    geometry_msgs::TwistStamped adaptationMsg;
    adaptationMsg.header.stamp = ros::Time::now();
    adaptationMsg.twist.linear.x = terrainVariance;
    adaptationMsg.twist.linear.y = lowPassFilteredTerrainContinuityValue_;
    adaptationMsg.twist.angular.x = supportSurfaceUncertaintyEstimation;
    adaptationMsg.twist.angular.y = getMeanGroundTruthDifference();
    adaptationMsg.twist.angular.z = sinkageVariance;
    adaptationMsg.twist.linear.z = 0.0; // Still to use..

    // Get the foot Tip Position.
    Position3 footTip3 = getFootTipPosition3(tip);
    Position footTip(footTip3(0), footTip3(1));


    const ros::WallDuration duration1 = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN::::::::::::: GP NOTILE regression initialization was done in %f s.", duration1.toSec());


    // Iterate through the model tiles.
    //for (int i = -noOfTilesPerHalfSide; i <= noOfTilesPerHalfSide; ++i){
    //    for (int j = -noOfTilesPerHalfSide; j <= noOfTilesPerHalfSide; ++j){
    //        if (sqrt(pow(i * tileResolution,2) + pow(j * tileResolution,2)) < radius){

    // GP Parameters -> to move outside of the loop and apply the data clear method (TODO!)
    int inputDim = 2;
    int outputDim = 1;
    //double radius = 0.6;
    GaussianProcessRegression<float> myGPR(inputDim, outputDim);
    myGPR.SetHyperParams(GPLengthscale_, GPSigmaN_, GPSigmaF_);

    // Get the tile position.
   // Position posTile;
   // posTile(0) = footTip(0) + i * tileResolution;
   // posTile(1) = footTip(1) + j * tileResolution;

    // Loop to add training data to GP regression.
    for (CircleIterator iterator(supportMap, footTip, radius); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);

        // Get Position of cell and set as input data for GP training.
        Position cellPos;
        supportMap.getPosition(index, cellPos); // This is wrong!!!!!!!!
        trainInput(0) = cellPos(0);
        trainInput(1) = cellPos(1);

        // Set Difference between sinkage depth layer and elevation layer as training output for GP.
        trainOutput(0) = rawMap.at("elevation", index) - supportMap.at("sinkage_depth_gp", index);

        // SomeTests
        //Eigen::VectorXf trainOutput2(outputDim);
        //trainOutput2(0) = supportMap.at("elevation", index);

        // Probability sampling and replace the training data by plane value
        bool insert = sampleContinuityPlaneToTrainingData(cellPos, footTip, lowPassFilteredTerrainContinuityValue_);
        if (insert) {
            trainOutput(0) = supportMap.at("terrain_continuity_gp", index);
        }

        // Add the training data to GP regression.
        if (!isnan(trainOutput(0))) myGPR.AddTrainingData(trainInput, trainOutput);
    }
    //std::cout << "The Number of Data in this tile is: " << myGPR.get_n_data() << std::endl;

    // Only perform regression if no. of Data is sufficiently high.
    if (myGPR.get_n_data() > 1){

        // Loop here to get test output
        for (CircleIterator iterator(supportMap, footTip, radius); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            auto& supportMapElevationGP = supportMap.at("elevation_gp", index);
            auto& supportMapVarianceGP = supportMap.at("variance_gp", index);

            // Prepare test input.
            Eigen::VectorXf testInput(inputDim);
            Position inputPosition;
            supportMap.getPosition(index, inputPosition);
            testInput(0) = inputPosition(0);
            testInput(1) = inputPosition(1);

            // Perform regression.
            supportMapElevationGP = myGPR.DoRegression(testInput)(0);
            //supportMapElevationGP = myGPR.DoRegressionVariance(testInput)(0);
            supportMapVarianceGP = 0.0;
            //supportMapVarianceGP = fabs(myGPR.GetVariance()(0));

            //if (isnan(supportMapElevationGP)) supportMapElevationGP = regressionOutput;
            //else supportMapElevationGP = 0.5 *  supportMapElevationGP + regressionOutput * 0.5;

            //if (isnan(supportMapVarianceGP)) supportMapVarianceGP = regressionOutputVariance;
            //else supportMapVarianceGP = 0.5 * supportMapVarianceGP + regressionOutputVariance * 0.5;
        }
    }
    // Visualization of tile centers.
    //geometry_msgs::Point p;
    //p.x = posTile(0);
    //p.y = posTile(1);
    //p.z = 0.0;
    //tileMarkerList_.points.push_back(p);
    //supportSurfaceAddingAreaPublisher_.publish(tileMarkerList_);
   // myGPR.ClearTrainingData();
   //         }
   //     }
   // }


    const ros::WallDuration duration2 = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN:::::::::iterated NOTILE through no tiles in %f s.", duration2.toSec());


    // Adding procedure.
    for (CircleIterator iterator(supportMap, footTip, radius); !iterator.isPastEnd(); ++iterator) { // HAcked to raw map for testing..
        const Index index(*iterator);

        Position addingPosition;
        supportMap.getPosition(index, addingPosition);
        double distance = sqrt(pow(addingPosition(0) - footTip(0), 2) +
                               pow(addingPosition(1) - footTip(1), 2));

        // Old weighting scheme.
        //double distanceFactor = sqrt(pow(addingPosition(0) - footTip(0), 2) +
        //                                   pow(addingPosition(1) - footTip(1), 2)) / radius;

        // New weighting scheme here:
        double weight;
        if (distance <= 0.2 * radius) weight = 1.0 - (distance / radius);
        else if (distance >= 0.2 * radius && distance <= weightDecayThreshold_ * radius) weight = 0.8;
        else weight = weightDecayThreshold_ - weightDecayThreshold_ * (distance - weightDecayThreshold_ * radius)
                / ((1.0 - weightDecayThreshold_) * radius);
        // End of new weighting scheme

        // Get the various layers.
        auto& supportMapElevationGP = supportMap.at("elevation_gp", index);
        auto& supportMapElevationGPAdded = supportMap.at("elevation", index);
        auto& supportMapVarianceGP = supportMap.at("variance_gp", index);
        auto& supportMapVarianceGPAdded = supportMap.at("variance", index);

        //
        if (!isnan(supportMapElevationGPAdded)) {
            supportMapElevationGPAdded = (1 - weight) * supportMapElevationGPAdded + (supportMapElevationGP) * weight;
        }
        else {
            supportMapElevationGPAdded = supportMapElevationGP;
        }

        if (!isnan(supportMapVarianceGPAdded)) {
            supportMapVarianceGPAdded = (1 - weight) * supportMapVarianceGPAdded + (supportMapVarianceGP) * weight;
        }
        else supportMapVarianceGPAdded = supportMapVarianceGP;

//        if (!useBag_){
//            auto& fusedMapElevationGP = fusedMap.at("elevation_gp_added_raw", index);
//            if (!isnan(supportMapElevationGPAdded)) fusedMapElevationGP = supportMapElevationGPAdded;
//        }
    }


    const ros::WallDuration duration = ros::WallTime::now() - mainGPStartTime;
    ROS_INFO("MAIN:::::::::::: GP NOTILE regression was run totally in %f s.", duration.toSec());


    // Publish adaptation Parameters
    varianceTwistPublisher_.publish(adaptationMsg);
    return true;
}


bool SupportSurfaceEstimation::simpleSinkageDepthLayer(std::string& tip, const double& tipDifference,
                                                       GridMap& supportMap, GridMap& rawMap){

    //if (tip == "left" || tip == "right") {
        if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0 && leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0){

            if (!isnan(tipDifference)) {

                double radius = 1.2;
                int maxSizeFootTipHistory;
                if (runHindLegSupportSurfaceEstimation_) maxSizeFootTipHistory = 20;
                else maxSizeFootTipHistory = 10;

                //std::cout << "TIP DIFFERENCE::::::::::::::::::::::::::::::::::::::::::::::: " << -tipDifference << std::endl;
                // DEBUG:
                //if (-tipDifference < 0.0) std::cout << "Attention, negative tip DIfference found!!: " << -tipDifference << std::endl;
                //if (initializedLeftSinkageDepth_ && leftFrontSinkageDepth_ < 0.0) std::cout << "Attention, negative leftfrontsd found!!: " << leftFrontSinkageDepth_ << std::endl;
                //if (initializedRightSinkageDepth_ && rightFrontSinkageDepth_ < 0.0) std::cout << "Attention, negative rightfrontsd found!!: " << rightFrontSinkageDepth_ << std::endl;
                //if (isnan(leftFrontSinkageDepth_)) std::cout << " NANANANANANANAANNNNNNNNNNNNANNNNNNNAAAAAAAAAAAAAANNNNNNNNNNNNNN" << std::endl;

                // Do history vector.
                Position3 footTip = getFootTipPosition3(tip);

                sinkageDepthPoints_.header.frame_id = "odom_drift_adjusted";
                bool pointCloudVersion = false;
                if (pointCloudVersion) {
                    geometry_msgs::Point32 point;
                    point.x = footTip(0);
                    point.y = footTip(1);
                    point.z = -tipDifference;
                    //pointcloud.data.push_back(point);
                    sinkageDepthPoints_.points.push_back(point);
                    sinkageDepthPoints_.header.frame_id = "odom_drift_adjusted";
                    if (sinkageDepthPoints_.points.size() > maxSizeFootTipHistory)
                        sinkageDepthPoints_.points.erase(sinkageDepthPoints_.points.begin());
                }

                // Pointcloud version of the foot contact points.

                sinkageDepthHistory_.push_back(-tipDifference);
                sinkageFootTipHistoryGP_.push_back(footTip);

//                if (isnan(tipDifference)) {
//                    if (tip == "left" && !isnan(leftFrontSinkageDepth_) && initializedLeftSinkageDepth_) {
//                        sinkageDepthHistory_.push_back(leftFrontSinkageDepth_); // Do not update the sinkage depth if there is no new information.
//                        sinkageFootTipHistoryGP_.push_back(footTip);
//                        if (fabs(leftFrontSinkageDepth_) < 0.001) std::cout << "WARNING: the stored sinkage depth value is very close to zero!!! \n \n \n \n \n WARNING \n";
//                    }
//                    if (tip == "right" && !isnan(rightFrontSinkageDepth_) && initializedRightSinkageDepth_) {
//                        sinkageDepthHistory_.push_back(rightFrontSinkageDepth_);
//                        sinkageFootTipHistoryGP_.push_back(footTip);
//                        if (fabs(rightFrontSinkageDepth_) < 0.001) std::cout << "WARNING: the stored sinkage depth value is very close to zero!!! \n \n \n \n \n WARNING \n";
//                    }
//                    if (tip == "lefthind" && !isnan(leftHindSinkageDepth_) && initializedLeftHindSinkageDepth_) {
//                        sinkageDepthHistory_.push_back(leftHindSinkageDepth_);
//                        sinkageFootTipHistoryGP_.push_back(footTip);
//                        if (fabs(leftHindSinkageDepth_) < 0.001) std::cout << "WARNING: the stored sinkage depth value is very close to zero!!! \n \n \n \n \n WARNING \n";
//                    }
//                    if (tip == "righthind" && !isnan(rightHindSinkageDepth_) && initializedRightHindSinkageDepth_) {
//                        sinkageDepthHistory_.push_back(rightHindSinkageDepth_);
//                        sinkageFootTipHistoryGP_.push_back(footTip);
//                        if (fabs(rightHindSinkageDepth_) < 0.001) std::cout << "WARNING: the stored sinkage depth value is very close to zero!!! \n \n \n \n \n WARNING \n";
//                    }

//                }
//                else {
//                    sinkageDepthHistory_.push_back(-tipDifference);
//                    sinkageFootTipHistoryGP_.push_back(footTip);
//                    if (tip == "left"){
//                        leftFrontSinkageDepth_ = -tipDifference;
//                        initializedLeftSinkageDepth_ = true;
//                    }
//                    if (tip == "right") {
//                        rightFrontSinkageDepth_ = -tipDifference;
//                        initializedRightSinkageDepth_ = true;
//                    }
//                    if (tip == "lefthind") {
//                        leftHindSinkageDepth_ = -tipDifference;
//                        initializedLeftHindSinkageDepth_ = true;
//                    }
//                    if (tip == "righthind") {
//                        rightHindSinkageDepth_ = -tipDifference;
//                        initializedRightHindSinkageDepth_ = true;
//                    }

//                }
                //std::cout << "leftFrontSinkageDepth:: " << leftFrontSinkageDepth_ << " rightFrontSinkageDepth_ " << rightFrontSinkageDepth_ << std::endl;


                if (sinkageFootTipHistoryGP_.size() > maxSizeFootTipHistory) { // They must be the same!!
                    sinkageFootTipHistoryGP_.erase(sinkageFootTipHistoryGP_.begin());
                    sinkageDepthHistory_.erase(sinkageDepthHistory_.begin());
                    //sinkageDepthHistoryHind_.erase(sinkageDepthHistoryHind_.begin());
                }

                if (sinkageFootTipHistoryGP_.size() != sinkageDepthHistory_.size()) {
                    std::cout << "Attention, having issues \n \n \n \n ISSUES i said!!" << std::endl;
                }

                Position center(footTip(0), footTip(1));

                for (grid_map::CircleIterator iterator(supportMap, center, radius); !iterator.isPastEnd(); ++iterator) {

                    const Index index(*iterator);
                    auto& supportMapElevationGPSinkage = supportMap.at("sinkage_depth_gp", index);
                    auto& supportMapVarianceGPSinkage = supportMap.at("sinkage_depth_variance_gp", index);

                    Position pos;
                    supportMap.getPosition(index, pos);

                    float addingDistance = sqrt(pow(pos(0) - footTip(0), 2) + pow(pos(1) - footTip(1), 2));

                    // Keep track of total weight and temporary height value.
                    float totalWeight = 0.0;
                    float tempHeight = 0.0;

                    // TODO: Publish them for rviz visualization!
                    if (pointCloudVersion) {
                        int maxIter = sinkageDepthPoints_.points.size();
                        for (unsigned int i = 0; i < maxIter; ++i) {
                            double distance = sqrt(pow(sinkageDepthPoints_.points[i].x - footTip(0), 2) + pow(sinkageDepthPoints_.points[i].y - footTip(1), 2));
                            if (distance < radius) {
                                float localDistance = sqrt(pow(sinkageDepthPoints_.points[i].x - pos(0), 2) + pow(sinkageDepthPoints_.points[i].y - pos(1), 2));
                                float weight;
                                if (localDistance > 0.0001) weight = 1 / pow(localDistance, exponentSinkageDepthWeight_); // Hacking here to test the range of the power.
                                else weight = 0.0;
                                totalWeight += weight;
                                if (!isnan(sinkageDepthPoints_.points[i].z)) tempHeight += sinkageDepthPoints_.points[i].z * weight;
                            }
                        }
                    }

                    else {
                        int maxIter = min(sinkageFootTipHistoryGP_.size(), sinkageDepthHistory_.size());


                        for (unsigned int i = 0; i < maxIter; ++i) {
                            double distance = sqrt(pow(sinkageFootTipHistoryGP_[i](0) - footTip(0), 2) + pow(sinkageFootTipHistoryGP_[i](1) - footTip(1), 2));
                            if (distance < radius) {
                                float localDistance = sqrt(pow(sinkageFootTipHistoryGP_[i](0) - pos(0), 2) + pow(sinkageFootTipHistoryGP_[i](1) - pos(1), 2));
                                float weight;
                                if (localDistance > 0.0001) weight = 1 / pow(localDistance, exponentSinkageDepthWeight_); // Hacking here to test the range of the power.
                                else weight = 0.0;
                                totalWeight += weight;
                                if (!isnan(sinkageDepthHistory_[i])) tempHeight += sinkageDepthHistory_[i] * weight;
                            }
                        }
                    }



                    double output;
                    if (totalWeight > 0.0001) output = tempHeight / totalWeight;
                    else if (!isnan(tipDifference)) output = tipDifference;
                    else output = 0.0;

                    // TODO: use the nonsimple weighting scheme here..

                    float addingWeight = fmax(1 - (addingDistance / radius), 0.0);

                    if (addingDistance <= weightDecayThreshold_ * radius) addingWeight = 1.0 - (1.0 - weightDecayThreshold_) * (addingDistance / radius);
                    else addingWeight = weightDecayThreshold_ - weightDecayThreshold_ * (addingDistance - weightDecayThreshold_ * radius)
                            / ((1.0 - weightDecayThreshold_) * radius);

                    if (!isnan(output)){

                        if (!isnan(supportMapElevationGPSinkage))  // Hacked away, attention!!
                            supportMapElevationGPSinkage = addingWeight * output + (1 - addingWeight) * supportMapElevationGPSinkage;
                        else supportMapElevationGPSinkage = output;
                    }

                    if (!isnan(fabs(supportMap.at("elevation", index) - rawMap.at("elevation", index))))
                        supportMapVarianceGPSinkage = fabs(supportMap.at("elevation", index) - rawMap.at("elevation", index)) - supportMapElevationGPSinkage;
                    else supportMapVarianceGPSinkage = 0.0;


                    //else std::cout << "Output would have been NANANANANANANANANANANA" << std::endl;
                    // TODO: idea: exponent of distance proportionality as learning parameter.
                    //if (isnan(supportMap.at("terrain_continuity_gp", index)))
                }
            }
        }

    //}
    return true;
}

bool SupportSurfaceEstimation::sinkageDepthLayerGP(std::string& tip, const double& tipDifference, GridMap& supportMap){

    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0 && leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0){

        double radius = 1.2;
        int maxSizeFootTipHistory;
        if (runHindLegSupportSurfaceEstimation_) maxSizeFootTipHistory = 20;
        else maxSizeFootTipHistory = 10;

        // Do history vector.
        Position3 footTip = getFootTipPosition3(tip);

        if (!isnan(tipDifference)) {
            sinkageDepthHistory_.push_back(-tipDifference);
            sinkageFootTipHistoryGP_.push_back(footTip);
        }

        if (sinkageFootTipHistoryGP_.size() > maxSizeFootTipHistory) { // They must be the same!!
            sinkageFootTipHistoryGP_.erase(sinkageFootTipHistoryGP_.begin());
            sinkageDepthHistory_.erase(sinkageDepthHistory_.begin());
            //sinkageDepthHistoryHind_.erase(sinkageDepthHistoryHind_.begin());
        }

        if (sinkageFootTipHistoryGP_.size() != sinkageDepthHistory_.size()) {
            std::cout << "Attention, having issues \n \n \n \n ISSUES i said!!" << std::endl;
        }

        //std::cout << "here i was still 1" << std::endl;

        int inputDim = 2;
        int outputDim = 1;
        GaussianProcessRegression<float> sinkageGPR(inputDim, outputDim);

        // lengthscale, sigma_n, sigma_f, betaNN, a_RQ, cConst, kernel
        //sinkageGPR.SetHyperParams(sinkageGPLengthscale_, sinkageGPSigmaN_, sinkageGPSigmaF_);
        sinkageGPR.SetHyperParamsAll(sinkageGPLengthscale_, 0.1, sinkageGPSigmaN_, sinkageGPSigmaF_,
                                        continuityGPNNBeta_, continuityGPRQa_, continuityGPCC_, continuityHYa_, continuityHYb_,
                                        sinkageGPKernel_);

        double totalHeight = 0.0;
        int counter = 0;
        for (unsigned int j = 0; j < sinkageFootTipHistoryGP_.size(); ++j) {
            Position tipPosLoc(sinkageFootTipHistoryGP_[j](0), sinkageFootTipHistoryGP_[j](1));
            double localDistance = sqrt(pow(tipPosLoc(0) - footTip(0), 2) + pow(tipPosLoc(1) - footTip(1), 2));
            if (localDistance < radius + 0.3) {
                // To calculate the mean.
                totalHeight += sinkageDepthHistory_[j];
                counter++;
            }
        }

        double meanSinkageGP = totalHeight / double(counter);


        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);

        for (unsigned int j = 0; j < sinkageFootTipHistoryGP_.size(); ++j) {

            Position tipPosLoc(sinkageFootTipHistoryGP_[j](0), sinkageFootTipHistoryGP_[j](1));
            double localDistance = sqrt(pow(tipPosLoc(0) - footTip(0), 2) + pow(tipPosLoc(1) - footTip(1), 2));

            if (localDistance < radius + 0.3) {
                trainInput(0) = tipPosLoc(0);
                trainInput(1) = tipPosLoc(1);
                trainOutput(0) = sinkageDepthHistory_[j] - meanSinkageGP;
                if (!isnan(trainOutput(0))) sinkageGPR.AddTrainingData(trainInput, trainOutput);
            }
        }

        double l, n, f;
        sinkageGPR.GetHyperParams(l, n, f);
        //std::cout << "hyperparams: l: " << l << " n: " << n << " f: " << f << std::endl;


        if (sinkageGPR.get_n_data() > 0) {
            Position center(footTip(0), footTip(1));
            for (grid_map::CircleIterator iterator(supportMap, center, radius); !iterator.isPastEnd(); ++iterator) {

                const Index index(*iterator);
                auto& supportMapElevationGPSinkage = supportMap.at("sinkage_depth_gp", index);
                auto& supportMapElevationGPSinkageVariance = supportMap.at("sinkage_depth_variance_gp", index);

                Position pos;
                supportMap.getPosition(index, pos);
                Eigen::VectorXf testInput(inputDim);
                testInput(0) = pos(0);
                testInput(1) = pos(1);

                double regressionOutput = sinkageGPR.DoRegressionNNVariance(testInput)(0) + meanSinkageGP;
                double regressionOutputVariance = sinkageGPR.GetVariance()(0);

                double distance = sqrt(pow(pos(0) - footTip(0), 2) + pow(pos(1) - footTip(1), 2));
                double weight;
                if (distance <= weightDecayThreshold_ * radius) weight = 1.0 - (1.0 - weightDecayThreshold_) * (distance / radius);
                else weight = weightDecayThreshold_ - weightDecayThreshold_ * (distance - weightDecayThreshold_ * radius)
                        / ((1.0 - weightDecayThreshold_) * radius);

                // Adjust this!!
                if (!isnan(regressionOutput)){
                    if (isnan(supportMapElevationGPSinkage) || !supportMap.isValid(index)){
                        supportMapElevationGPSinkage = regressionOutput;

                    }
                    else{
                        if (!isnan(supportMapElevationGPSinkage))
                            supportMapElevationGPSinkage = (weight) * regressionOutput + (1.0 - weight) * supportMapElevationGPSinkage; // Attention hacked this in here..
                    }
                }

                if (supportMapElevationGPSinkage < 0.0) supportMapElevationGPSinkage = 0.0;

                if (!isnan(regressionOutputVariance)){
                    if (isnan(supportMapElevationGPSinkageVariance)){
                        supportMapElevationGPSinkageVariance = regressionOutputVariance;
                    }
                    else{
                        if (!isnan(supportMapElevationGPSinkageVariance))
                            supportMapElevationGPSinkageVariance = (weight) * regressionOutputVariance + (1.0 - weight) * supportMapElevationGPSinkageVariance; // Attention hacked this in here..
                    }
                }
            }
        }
    }
    return true;
}

// Put them together at some time..
bool SupportSurfaceEstimation::simpleTerrainContinuityLayer(std::string& tip, GridMap& supportMap){

    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0){

        Eigen::Vector3f tipLeftVec = getLatestLeftStance();
        Eigen::Vector3f tipRightVec = getLatestRightStance();
        Position3 tipLeftPos3(tipLeftVec(0), tipLeftVec(1), tipLeftVec(2));
        Position3 tipRightPos3(tipRightVec(0), tipRightVec(1), tipRightVec(2));

        double radius = 0.7;
        int maxSizeFootTipHistory = 15;

        // Do history vector.
        Position3 footTip;
        if (tip == "left") footTip = tipLeftPos3;
        if (tip == "right") footTip = tipRightPos3;


        if (!isnan(tipLeftVec(2)) && !isnan(tipRightVec(2))) {
            footTipHistoryGP_.push_back(footTip);
            if (footTipHistoryGP_.size() > maxSizeFootTipHistory) footTipHistoryGP_.erase(footTipHistoryGP_.begin()); // This was missing, should speed things up..

            Position center(footTip(0), footTip(1));

            for (CircleIterator iterator(supportMap, center, radius); !iterator.isPastEnd(); ++iterator) {

                const Index index(*iterator);
                auto& supportMapElevationGPContinuity = supportMap.at("terrain_continuity_gp", index);
                Position pos;
                supportMap.getPosition(index, pos);
                float addingDistance = sqrt(pow(pos(0) - footTip(0), 2) + pow(pos(1) - footTip(1), 2));

                // Keep track of total weight and temporary height value.
                float totalWeight = 0.0;
                float tempHeight = 0.0;

                for (unsigned int i = 0; i < footTipHistoryGP_.size(); ++i) {
                    double distance = sqrt(pow(footTipHistoryGP_[i](0) - footTip(0), 2) + pow(footTipHistoryGP_[i](1) - footTip(1), 2));
                    if (distance < radius) {
                        if (i > 2) {
                            Eigen::Vector4f coeffs = getPlaneCoeffsFromThreePoints(footTipHistoryGP_[i-2], footTipHistoryGP_[i-1], footTipHistoryGP_[i]);
                            double planeHeight = evaluatePlaneFromCoefficients(coeffs, pos);


                            double triangleArea = get2DTriangleArea(footTipHistoryGP_[i-2], footTipHistoryGP_[i-1], footTipHistoryGP_[i]);

                            //std::cout << "Crazy triangle area overflow:!! " << triangleArea << std::endl;
                            // TODO: Evaluate Plane spun by three points and get do the same, but with the point value, that would have been gained by the plane

                            // An issue is here if trotting on the spot!! then may get very steep!!
                            // TODO TODO TODO!!!

                            float localDistance = sqrt(pow(footTipHistoryGP_[i](0) - pos(0), 2) + pow(footTipHistoryGP_[i](1) - pos(1), 2));
                            float weight;
                            if (localDistance > 0.0001) weight = 1.0 / pow(localDistance, exponentTerrainContinuityWeight_); // Hacking here to test the range of the power.
                            else weight = 0.0;
                            weight = weight * triangleArea; // Test, weigh according to projected triangle area..

                            totalWeight += weight;
                            if (!isnan(planeHeight)) tempHeight += planeHeight * weight; // Hackin around in here, make clean
                        }
                    }
                }

                double output;
                if (totalWeight > 0.0001) output = tempHeight / totalWeight;
                else output = footTip(2); // Here is a problem, get it very neat!!! TODO! -> does this solve it????

                float addingWeight = fmin(fmax(1.0 - (addingDistance / radius), 0.0), 1.0);
                if (!isnan(output)){
                    if (!supportMap.isValid(index)){
                        supportMapElevationGPContinuity = output;
                    }
                    else{
                        if (!isnan(supportMapElevationGPContinuity)) supportMapElevationGPContinuity = addingWeight * output + (1 - addingWeight) * supportMapElevationGPContinuity;
                    }
                }
            }
        }
    }
    return true;
}

bool SupportSurfaceEstimation::terrainContinuityLayerGP(std::string& tip, GridMap& supportMap, const double& totalEstimatedDrift){

    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0){

        // Set start time for time calculation for continuity layer update.
        const ros::WallTime continuityGPStartTime(ros::WallTime::now());

        Position3 tipPos3 = getFootTipPosition3(tip);
        Position tipPos(tipPos3(0), tipPos3(1));

        int maxSizeFootTipHistory;
        if (runHindLegSupportSurfaceEstimation_) maxSizeFootTipHistory = 18;
        else maxSizeFootTipHistory = 9;



        // Option to change into corrected frame..
        if (runDriftRefinementSupportSurface_) tipPos3(2) += 0.0 * totalEstimatedDrift; // Check if sensible.. TODO, cleaner concept for this!!
        footTipHistoryGP_.push_back(tipPos3);

        // PointCloud Version for frame transform.
        bool pointCloudVersion = true;
        if (pointCloudVersion) {
            geometry_msgs::Point32 point;
            point.x = tipPos3(0);
            point.y = tipPos3(1);
            point.z = tipPos3(2) - totalEstimatedDrift;  // TODO: think if this has to be adjusted by the total drift..
            continuityPoints_.points.push_back(point);
            if (continuityPoints_.points.size() > maxSizeFootTipHistory)
                continuityPoints_.points.erase(continuityPoints_.points.begin());
            continuityPoints_.header.frame_id = "odom_drift_adjusted";
        }


        if (footTipHistoryGP_.size() > maxSizeFootTipHistory)
            footTipHistoryGP_.erase(footTipHistoryGP_.begin());

        int inputDim = 2;
        int outputDim = 1;
        float radius = 0.7;
        if (sizeSupportSurfaceUpdate_ == "large") radius = 0.9;

        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);

        GaussianProcessRegression<float> continuityGPR(inputDim, outputDim);

        if (continuityGPKernel_ == "nn") {

        }
        else if (continuityGPKernel_ == "ou") {
            continuityGPNNLengthscale_ = continuityGPOULengthscale_;
            continuityGPNNSigmaN_ = continuityGPOUSigmaN_;
            continuityGPNNSigmaF_ = continuityGPOUSigmaF_;
        }
        else if (continuityGPKernel_ == "hy") {
            continuityGPNNSigmaN_ = continuityGPHYSigmaN_;
            continuityGPNNSigmaF_ = continuityGPHYSigmaF_;
        }
        else if (continuityGPKernel_ == "sqe") {
            continuityGPNNLengthscale_ = continuityGPLengthscale_;
            continuityGPNNSigmaN_ = continuityGPSigmaN_;
            continuityGPNNSigmaF_ = continuityGPSigmaF_;
        }
        else if (continuityGPKernel_ == "rq") {
            continuityGPNNLengthscale_ = continuityGPRQLengthscale_;
            continuityGPNNSigmaN_ = continuityGPRQSigmaN_;
            continuityGPNNSigmaF_ = continuityGPRQSigmaF_;
        }
        else if (continuityGPKernel_ == "ousqe") {
            continuityGPNNLengthscale_ = continuityGPOULengthscale_;
            continuityGPNNSigmaN_ = continuityGPOUSigmaN_;
            continuityGPNNSigmaF_ = continuityGPOUSigmaF_;
            continuityGPNNLengthscale2_ = continuityGPOUSQELengthscale_;
        }

       // std::cout << "lscale::::::::::::::::::::::::::::::::                                           :::::::::::::::::::::: "
         //         << continuityGPOUSQELengthscale_ << std::endl;


        // lengthscale, sigma_n, sigma_f, betaNN, a_RQ, cConst, kernel
        continuityGPR.SetHyperParamsAll(continuityGPNNLengthscale_, continuityGPOUSQELengthscale_, continuityGPNNSigmaN_, continuityGPNNSigmaF_,
                                        continuityGPNNBeta_, continuityGPRQa_, continuityGPCC_, continuityHYa_, continuityHYb_,
                                        continuityGPKernel_);
        //continuityGPR.SetHyperParams(continuityGPLengthscale_, continuityGPSigmaN_, continuityGPSigmaF_);

        double meanGP;

        if (pointCloudVersion) {
            double totalHeight = 0.0;
            int counter = 0;
            // Version of gaussian kernel with zero mean.
            for (unsigned int j = 0; j < continuityPoints_.points.size(); ++j) {
                Position tipPosLoc(continuityPoints_.points[j].x, continuityPoints_.points[j].y);
                double localDistance = sqrt(pow(tipPosLoc(0) - tipPos(0), 2) + pow(tipPosLoc(1) - tipPos(1), 2));

                if (localDistance < radius + 0.3) {
                    // To calculate the mean.
                    totalHeight += continuityPoints_.points[j].z;
                    counter++;
                }
            }
            meanGP = totalHeight / double(counter);

            for (unsigned int j = 0; j < continuityPoints_.points.size(); ++j) {
                Position tipPosLoc(continuityPoints_.points[j].x, continuityPoints_.points[j].y);
                double localDistance = sqrt(pow(tipPosLoc(0) - tipPos(0), 2) + pow(tipPosLoc(1) - tipPos(1), 2));

                if (localDistance < radius + 0.3) {
                    trainInput(0) = tipPosLoc(0);
                    trainInput(1) = tipPosLoc(1);
                    //trainOutput(0) = footTipHistoryGP_[j](2);
                    trainOutput(0) = continuityPoints_.points[j].z - meanGP;
                    continuityGPR.AddTrainingData(trainInput, trainOutput);
                }
            }
        }
        else {
            double totalHeight = 0.0;
            int counter = 0;
            // Version of gaussian kernel with zero mean.
            for (unsigned int j = 0; j < footTipHistoryGP_.size(); ++j) {
                Position tipPosLoc(footTipHistoryGP_[j](0), footTipHistoryGP_[j](1));
                double localDistance = sqrt(pow(tipPosLoc(0) - tipPos(0), 2) + pow(tipPosLoc(1) - tipPos(1), 2));

                if (localDistance < radius + 0.3) {
                    // To calculate the mean.
                    totalHeight += footTipHistoryGP_[j](2);
                    counter++;
                }
            }
            meanGP = totalHeight / double(counter);

            for (unsigned int j = 0; j < footTipHistoryGP_.size(); ++j) {
                Position tipPosLoc(footTipHistoryGP_[j](0), footTipHistoryGP_[j](1));
                double localDistance = sqrt(pow(tipPosLoc(0) - tipPos(0), 2) + pow(tipPosLoc(1) - tipPos(1), 2));

                if (localDistance < radius + 0.3) {
                    trainInput(0) = tipPosLoc(0);
                    trainInput(1) = tipPosLoc(1);
                    //trainOutput(0) = footTipHistoryGP_[j](2);
                    trainOutput(0) = footTipHistoryGP_[j](2) - meanGP;
                    continuityGPR.AddTrainingData(trainInput, trainOutput);
                }
            }
        }





        for (CircleIterator iterator(supportMap, tipPos, radius); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            Eigen::VectorXf testInput(inputDim);

            auto& supportMapContinuityGP = supportMap.at("terrain_continuity_gp", index);
            auto& supportMapContinuityVarianceGP = supportMap.at("terrain_continuity_variance_gp", index);

            Position cellPos;
            supportMap.getPosition(index, cellPos);
            testInput(0) = cellPos(0);
            testInput(1) = cellPos(1);

            //double outputHeight = continuityGPR.DoRegressionNN(testInput)(0);

            // Mean shifted version.
            double outputHeight = continuityGPR.DoRegressionNNVariance(testInput)(0) + meanGP;  // Hope this shouldnt be testOutput(0)
            double outputVariance = fabs(continuityGPR.GetVariance()(0));


            double distance = sqrt(pow(cellPos(0) - tipPos(0), 2) + pow(cellPos(1) - tipPos(1), 2));


            //float addingWeight;

            //addingWeight = fabs(fmax(1.0 - (localDistance / 3.0 * radius), 0.0));


            double weight;
            if (distance <= weightDecayThreshold_ * radius) weight = 1.0 - (1.0 - weightDecayThreshold_) * (distance / radius);
            else weight = weightDecayThreshold_ - weightDecayThreshold_ * (distance - weightDecayThreshold_ * radius)
                    / ((1.0 - weightDecayThreshold_) * radius);

            if (!isnan(outputHeight)){
                if (isnan(supportMapContinuityGP)){
                    supportMapContinuityGP = outputHeight;
                }
                else{
                    if (!isnan(supportMapContinuityGP))
                        supportMapContinuityGP = (weight) * outputHeight + (1.0 - weight) * supportMapContinuityGP; // Attention hacked this in here..
                }
            }

            if (pointCloudVersion) {
                double minDist = 10000;
                for (unsigned int j = 0; j < continuityPoints_.points.size(); ++j) {
                    Position tipPosLoc(continuityPoints_.points[j].x, continuityPoints_.points[j].y);
                    double distVar = sqrt(pow(tipPosLoc(0) - cellPos(0), 2) + pow(tipPosLoc(1) - cellPos(1), 2));
                    if (distVar < minDist) minDist = distVar;
                }
                supportMapContinuityVarianceGP = continuityVarianceFactor_ * minDist;
            }
            else {
                double minDist = 10000;
                for (unsigned int j = 0; j < footTipHistoryGP_.size(); ++j) {
                    Position tipPosLoc(footTipHistoryGP_[j](0), footTipHistoryGP_[j](1));
                    double distVar = sqrt(pow(tipPosLoc(0) - cellPos(0), 2) + pow(tipPosLoc(1) - cellPos(1), 2));
                    if (distVar < minDist) minDist = distVar;
                }
                supportMapContinuityVarianceGP = continuityVarianceFactor_ * minDist;
            }


//            if (!isnan(outputVariance)){
//                if (isnan(supportMapContinuityVarianceGP)){
//                    supportMapContinuityVarianceGP = distance;
//                }
//                else{
//                    if (!isnan(supportMapContinuityVarianceGP))
//                        supportMapContinuityVarianceGP = (weight) * distance + (1.0 - weight) * supportMapContinuityVarianceGP; // Attention hacked this in here..
//                }
//            }




        }
        const ros::WallDuration duration = ros::WallTime::now() - continuityGPStartTime;
        ROS_INFO("GP terrain continuity layer has been updated with a new point cloud in %f s.", duration.toSec());
    }

    return true;
}

bool SupportSurfaceEstimation::terrainContinuityLayerGPwhole(std::string& tip, GridMap& supportMap){

    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0){

        // Set start time for time calculation for continuity layer update.
        const ros::WallTime continuityGPStartTime(ros::WallTime::now());

        Position3 tipPos3 = getFootTipPosition3(tip);
        Position tipPos(tipPos3(0), tipPos3(1));

        int maxSizeFootTipHistory = 40;

        footTipHistoryGP_.push_back(tipPos3);
        if (footTipHistoryGP_.size() > maxSizeFootTipHistory)
            footTipHistoryGP_.erase(footTipHistoryGP_.begin());

        int inputDim = 2;
        int outputDim = 1;
        float radius = 0.7;

        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);

        GaussianProcessRegression<float> continuityGPR(inputDim, outputDim);
        continuityGPR.SetHyperParamsNN(continuityGPNNLengthscale_, continuityGPNNSigmaN_, continuityGPNNSigmaF_, continuityGPNNBeta_);; // just set some beta here..


        for (unsigned int j = 0; j < footTipHistoryGP_.size(); ++j) {


            Position cellPos(footTipHistoryGP_[j](0), footTipHistoryGP_[j](1));
            trainInput(0) = cellPos(0);
            trainInput(1) = cellPos(1);
            trainOutput(0) = footTipHistoryGP_[j](2);

            continuityGPR.AddTrainingData(trainInput, trainOutput);
        }



        for (GridMapIterator iterator(supportMap); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            Eigen::VectorXf testInput(inputDim);

            auto& supportMapContinuityGP = supportMap.at("terrain_continuity_gp", index);

            Position cellPos;
            supportMap.getPosition(index, cellPos);
            testInput(0) = cellPos(0);
            testInput(1) = cellPos(1);


            double outputHeight = continuityGPR.DoRegressionNN(testInput)(0);  // Hope this shouldnt be testOutput(0)
            supportMapContinuityGP = outputHeight;
        }

        const ros::WallDuration duration = ros::WallTime::now() - continuityGPStartTime;
        ROS_INFO("GP terrain continuity layer has been updated with a new point cloud in %f s.", duration.toSec());
    }
    return true;
}

bool SupportSurfaceEstimation::sampleContinuityPlaneToTrainingData(const Position& cellPos, const Position& center, const double& terrainContinuityValue){
    // Sample by functon with distance from the foot tip and add to the main GPR.

    // Distance.
    float distance = sqrt(pow(cellPos(0) - center(0), 2) + pow(cellPos(1) - center(1), 2));

    // Probability function of: (distance form tip, continuity estimation, opt: data amount)
    float prob = exp(-(terrainContinuityValue * distance));
    bool insertPlaneHeightAtCell;
    float r = ((float) rand() / (RAND_MAX));
    if (prob > r) insertPlaneHeightAtCell = true;
    else insertPlaneHeightAtCell = false;

    // (TODO!) Second version: weighted combination..

    // (TODO!) Third version: difference of foot tip plane and top layer added (weighted) to the sinkage depth layer (in front, i.e. walking direction)

    // (TODO!) Which is the best for variance calculation?

    return insertPlaneHeightAtCell;
}

Eigen::Vector4f SupportSurfaceEstimation::getPlaneCoeffsFromThreePoints(const Position3& Point1, const Position3& Point2, const Position3& Point3)
{
    Eigen::Vector3f Point1Eigen(Point1(0), Point1(1), Point1(2));
    Eigen::Vector3f Point2Eigen(Point2(0), Point2(1), Point2(2));
    Eigen::Vector3f Point3Eigen(Point3(0), Point3(1), Point3(2));
    Eigen::Vector3f vector1 = Point1Eigen - Point3Eigen;
    Eigen::Vector3f vector2 = Point2Eigen - Point3Eigen;
    Eigen::Vector3f normalVector = vector1.cross(vector2);
    Eigen::Vector3f normalisedNormalVector = normalVector.normalized();
    float d_val = - Point3Eigen.dot(normalisedNormalVector);
    Eigen::Vector4f coefficients(normalisedNormalVector(0), normalisedNormalVector(1), normalisedNormalVector(2), d_val);
    return coefficients;
}

double SupportSurfaceEstimation::evaluatePlaneFromCoefficients(const Eigen::Vector4f& coefficients, Position& cellPos){
    // a*x + b*y + c*z + d = 0
    // -> z = -(coeff(0) * cellPos(0) + coeff(1) * cellPos(1) + coeff(4)) / coeff(3)
    double planeElevation =  -(coefficients(0) * cellPos(0) + coefficients(1) * cellPos(1)
                               + coefficients(3)) / coefficients(2);
    return planeElevation;
}

bool SupportSurfaceEstimation::supportSurfaceUpperBoundingGP(GridMap& upperBoundMap, GridMap& supportSurfaceMap){

    Matrix& dataUpper = upperBoundMap["elevation"];
    Matrix& dataSup = supportSurfaceMap["elevation"];
    //Matrix& dataTip = supportSurfaceMap["elevation_gp_tip"];

    dataSup = dataUpper.cwiseMin(dataSup);
    //dataTip = dataUpper.cwiseMin(dataTip); // Hacked for testing
    return true;
}

double SupportSurfaceEstimation::getSupportSurfaceUncertaintyEstimation(){
    if(!isnan(supportSurfaceUncertaintyEstimation_)) return supportSurfaceUncertaintyEstimation_;
    else return 0.0;
}

double SupportSurfaceEstimation::getCumulativeSupportSurfaceUncertaintyEstimation(){
    if (!isnan(cumulativeSupportSurfaceUncertaintyEstimation_)) return cumulativeSupportSurfaceUncertaintyEstimation_;
    else return 0.0;
}

bool SupportSurfaceEstimation::testTrackMeanSupportErrorEvaluation(GridMap& supportMap){

    // Counter for cells to get the mean.
    int localCellCounter = 0;
    double localTotalDifference = 0.0;
    for (GridMapIterator iterator(supportMap); !iterator.isPastEnd(); ++iterator) {
        Index index = *iterator;
        Position cellPosition;
        supportMap.getPosition(index, cellPosition);
        auto& supportMapElevation = supportMap.at("elevation", index);
        double elevationDifference;

        if (cellPosition(1) <= 1.0 && cellPosition(1) >= -1.0) {
            if (!isnan(supportMapElevation)) {
                elevationDifference = getGroundTruthDifference(supportMap, cellPosition, index);
                localCellCounter++;
                localTotalDifference += elevationDifference;
            }
            // Avoid nans (ground truth layer is fori vsualizatoin purposes only)
            if (isnan(supportMap.at("ground_truth", index))) supportMap.at("ground_truth", index) = 0.0; // TODO: Not sound!!
            //std::cout << "supportMap.isvalid inside the updater!!!: " << supportMap.at("ground_truth", index) << std::endl;
        }
        else supportMap.at("ground_truth", index) = 0.0;
        //if (isnan(supportMap.at("ground_truth", index)))
            //std::cout << "supportMap.isvalid inside the updater!!!: " << supportMap.at("ground_truth", index) << std::endl;
    }
    if (localCellCounter > 0) {
        double meanElevationDifference = localTotalDifference / localCellCounter;
        overallConsideredStanceCounter_++;
        //std::cout << "global cell counter: " << overallConsideredStanceCounter_ << std::endl;
        overallSummedMeanElevationDifference_ += meanElevationDifference;
        std::cout << "overall Deviance: " << overallSummedMeanElevationDifference_ << std::endl;
        if (overallConsideredStanceCounter_ > 0) overallMeanElevationDifference_ =
                overallSummedMeanElevationDifference_ / overallConsideredStanceCounter_;
    }
    return true;
}

double SupportSurfaceEstimation::getGroundTruthDifference(GridMap& supportMap, Position cellPosition, Index index){
    double Difference = 0.0;
    double heightGroundTruth = 0.0;

    auto& supGroundTruth = supportMap.at("ground_truth", index);

    if (cellPosition(0) < 4.978663502) heightGroundTruth = 0.0;

    // First slope up.
    if (cellPosition(0) < 5.93200946738 && cellPosition(0) >= 4.978663502)  // Probably here is an error.. double chack!!!
        heightGroundTruth = (cellPosition(0) - 4.978663502) / (5.93200946738 - 4.978663502) * 0.1957601 + 0.004;

    // First Plane Horizontal.
    if (cellPosition(0) < 6.93200946738 && cellPosition(0) >= 5.93200946738)
        heightGroundTruth = 0.1957601 + 0.005;

    // Second slope down.
    if (cellPosition(0) < 7.452009511 && cellPosition(0) >= 6.93200946738)
        heightGroundTruth = 0.1957601 - ((cellPosition(0) - 6.93200946738) / (7.452009511 - 6.93200946738) * 0.0917601) + 0.004;

    // Third slope up
    if (cellPosition(0) < 7.96200946738 && cellPosition(0) >= 7.452009511)
        heightGroundTruth = (0.1957601 - 0.0917601) + ((cellPosition(0) - 7.452009511) / (7.96200946738 - 7.452009511) * 0.0917601) + 0.004;

    // Second Plane Horizontal.
    if (cellPosition(0) < 8.96200946738 && cellPosition(0) >= 7.96200946738)
        heightGroundTruth = 0.1957601 + 0.005;

    // Fourth slope down.
    if (cellPosition(0) < 9.914009467 && cellPosition(0) >= 8.96200946738)  // Here is definitely an error, double the distance
        heightGroundTruth = 0.1957601 - (cellPosition(0) - 8.96200946738) / (9.914009467 - 8.96200946738) * 0.1957601 + 0.004;

    if (cellPosition(0) < 11.0 - 0.005 && cellPosition(0) >= 9.450009467)
        heightGroundTruth = 0.0;

    if (cellPosition(0) < 12.0 + 0.005 && cellPosition(0) >= 11.0 - 0.005)
        heightGroundTruth = 0.1 + 0.005;

    if (cellPosition(0) < 13.0 - 0.005 && cellPosition(0) >= 12.0 + 0.005)
        heightGroundTruth = 0.0;

    if (cellPosition(0) < 14.0 - 0.005 && cellPosition(0) >= 13.0 - 0.005)
        heightGroundTruth = 0.1 + 0.005;

    if (cellPosition(0) < 15.0 && cellPosition(0) >= 13.0 - 0.005)
        heightGroundTruth = 0.0;


    //if (!isnan(heightGroundTruth)) supGroundTruth = (double)heightGroundTruth;
    if (isfinite(heightGroundTruth)) supGroundTruth = (double)heightGroundTruth;
    //supGroundTruth = 0.001 * (index(0) + index(1));
    //std::cout << "supportMap.isvalid: " << supportMap.at("ground_truth", index) << std::endl;

    Difference = fabs(supportMap.atPosition("elevation", cellPosition) - heightGroundTruth);

    if (cellPosition(0) > 15.0) {
        Difference = 0.0;
        std::cout << "EVALUATION GOALLINE REACHED!!!!!!" << std::endl;
    }

    return Difference;
}

double SupportSurfaceEstimation::getMeanGroundTruthDifference(){
    return overallMeanElevationDifference_;
}

double SupportSurfaceEstimation::get2DTriangleArea(const grid_map::Position3& Point1, const grid_map::Position3& Point2, const grid_map::Position3& Point3){

    return fabs(((Point2(0) - Point1(0))*(Point3(1) - Point1(1)) - (Point3(0) - Point1(0))*(Point2(1) - Point1(1)))/2.0);
}

double SupportSurfaceEstimation::signSelectiveLowPassFilter(double lowPassFilteredValue, double newValue, double filterGainDown, double filterGainUp){

   // std::cout << "lowPassFilteredValue:     ..........................: " << lowPassFilteredValue << std::endl;

    // Low pass filter (sign selective.)
    if (lowPassFilteredValue < newValue) lowPassFilteredValue = filterGainUp * lowPassFilteredValue + (1 - filterGainUp) * newValue;
    else lowPassFilteredValue = filterGainDown * lowPassFilteredValue + (1 - filterGainDown) * newValue;

  //  std::cout << "lowPassFilteredValue:     ..........................: " << lowPassFilteredValue << std::endl;
    return lowPassFilteredValue;
}

} /* namespace */
