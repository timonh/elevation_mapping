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

  // Filter chain initialization.
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("/grid_map_filter_chain_one"));
  if(!filterChain_.configure("/grid_map_filter_chain_one", nodeHandle_)){
      std::cout << "Could not configure the filter chain!!" << std::endl;
      return;
  }
  if(!filterChain2_.configure("/grid_map_filter_chain_two", nodeHandle_)){
      std::cout << "Could not configure the filter chain!!" << std::endl;
      return;
  }

  setParameters();

  //initialTime_ = ros::Time::now();
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
    nodeHandle_.param("use_sign_selective_continuity_filter", useSignSelectiveContinuityFilter_, true);
    nodeHandle_.param("sign_selective_continuity_filter_gain", signSelectiveContinuityFilterGain_, 0.5);

    // Initializations.
    supportSurfaceInitializationTrigger_ = false;
    cumulativeSupportSurfaceUncertaintyEstimation_ = 0.0;
    leftFrontSinkageDepth_ = 0.0;
    rightFrontSinkageDepth_ = 0.0;

    // Initializations for ground truth comparison in simulation.
    if (!useBag_) {
        std::cout << "useBag_: " << useBag_ << std::endl;
        overallConsideredStanceCounter_ = 0;
        overallSummedMeanElevationDifference_ = 0.0;
        overallMeanElevationDifference_ = 0.0;

    }
}

bool SupportSurfaceEstimation::updateSupportSurfaceEstimation(std::string tip, GridMap& rawMap,
                                                              GridMap& supportMap, GridMap& fusedMap, Eigen::Vector3f& stance){
    Position tipPosition(stance(0), stance(1));
    if(rawMap.isInside(tipPosition)) {



    proprioceptiveRoughnessEstimation(tip, stance);

        if (tip != "lefthind" && tip != "righthind" || runHindLegSupportSurfaceEstimation_
                && leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0) {

            //if (tip != "lefthind" && tip != "righthind") simpleTerrainContinuityLayer(tip, supportMap);
            //if (tip != "lefthind" && tip != "righthind")
            terrainContinuityLayerGP(tip, supportMap);
            // At som point: divide into 2 fcts.. TODO

            // Run only if stance vectors are initialized..
            if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0) {

                // Smoothed Top Layer to get vertical Difference between top layer and foot tip position (sinkage depth)
                setSmoothedTopLayer(tip, rawMap, supportMap);
                double verticalDifference = getFootTipElevationMapDifferenceGP(tip, supportMap);

                // Sinkage depth variance estimation.
                if (tip != "lefthind" && tip != "righthind") sinkageDepthVarianceEstimation(tip, verticalDifference);
                //! Yes all this!

                // Uncertainty Estimation based on low pass filtered error between foot tip height and predicted support Surface.
                if (tip == "left" || tip == "right") setSupportSurfaceUncertaintyEstimation(tip, supportMap); //! TODO: uncomment whats inside of this function.

                // Main gaussian process regression.
                mainGPRegression(tileResolution_, tileDiameter_, sideLengthAddingPatch_,
                                 tip, verticalDifference, rawMap, supportMap, fusedMap);
            }
        }

        // Ground truth difference evaluation for simulation environment.
        if (!useBag_) testTrackMeanSupportErrorEvaluation(supportMap);
        std::cout << "useBag_: " << useBag_ << std::endl;
    }

    ROS_WARN("Foot tip considered not to be inside the valid area of the elevation map. \n");

    // Set the raw Elevation Map layer to be the upper bound of the support surface estimation.
    supportSurfaceUpperBoundingGP(rawMap, supportMap);

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

bool SupportSurfaceEstimation::setSupportSurfaceUncertaintyEstimation(std::string tip, GridMap& supportMap){

    Eigen::Vector3f latestTip;
    if (tip == "left") latestTip = getLatestLeftStance();
    if (tip == "right") latestTip = getLatestRightStance();

    // So far not low pass filtered!! -> create Vector for that (TODO)
    grid_map::Position latestTipPosition(latestTip(0), latestTip(1));
    double diff = fabs(latestTip(2) - supportMap.atPosition("elevation_gp_added", latestTipPosition));
    supportSurfaceUncertaintyEstimation_ = diff;
    if (!isnan(diff)) cumulativeSupportSurfaceUncertaintyEstimation_ += diff;
    return true;
}

double SupportSurfaceEstimation::getFootTipElevationMapDifferenceGP(std::string tip, GridMap& supportMap){

    double radius = 0.7; // Maximum search radius for spiralling search in order to find the closest map element in case if nan is present..
    Position3 footTip3;
    footTip3 = getFootTipPosition3(tip);

    Position footTipHorizontal(footTip3(0), footTip3(1));
    double verticalDifference;
    std::cout << "exists ? foot Tip3 (2)" << footTip3(2) << std::endl;
    if (supportMap.exists("smoothed_top_layer_gp")) { // New Check!!!
        std::cout << "exists !" << std::endl;
        if(supportMap.isInside(footTipHorizontal)){
            std::cout << "went inside here check 1 " << std::endl;
            if (!isnan(supportMap.atPosition("smoothed_top_layer_gp", footTipHorizontal))){
                verticalDifference = footTip3(2) - supportMap.atPosition("smoothed_top_layer_gp", footTipHorizontal); // Hacked to rawMap
            }
            else verticalDifference = getClosestMapValueUsingSpiralIteratorElevation(supportMap, footTipHorizontal, radius, footTip3(2)); // New experiment.. Wrong, not difference yet!!!
        }
    }
    else verticalDifference = std::numeric_limits<double>::quiet_NaN();
    std::cout << "Vertical Difference!! -> ->: " << verticalDifference << std::endl;
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
            std::cout << "RETURNED DIFFERENCE TO A CLOSE NEIGHBOR USING SPIRALLING!!!" << std::endl;
            return tipHeight - MapReference.at("smoothed_top_layer_gp", index); // && MapReference.isValid(index)
        }
        counter++;
        if (counter > 11) break;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool SupportSurfaceEstimation::sinkageDepthVarianceEstimation(std::string tip, double verticalDifference){

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

bool SupportSurfaceEstimation::setSmoothedTopLayer(std::string tip, GridMap& rawMap, GridMap& supportMap){

    // Smoothing radius of area considered for sinkage depth calculation.
    double smoothingRadius = 0.09;

    // Get the foot tip position.
    Position3 tipPos3 = getFootTipPosition3(tip);
    Position footTipPosition(tipPos3(0), tipPos3(1));

    // Gaussian Process Regression Parameters.
    int inputDim = 2;
    int outputDim = 1;
    GaussianProcessRegression<float> smoothedTopLayerGPR(inputDim, outputDim);
    smoothedTopLayerGPR.SetHyperParams(1.0, 0.1, 0.001);

    // Iterate around the foot tip position to add Data
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

    if (smoothedTopLayerGPR.get_n_data() > 0){
        for (CircleIterator iterator(supportMap, footTipPosition, smoothingRadius); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);

            Eigen::VectorXf testInput(inputDim);
            Eigen::VectorXf testOutput(outputDim);
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


    std::cout << "tip inside proprioceptive: " << tip << std::endl;

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

    std::cout << "tip: " << tip << " diff: " << diff << std::endl;


    if (tip == "left" || tip == "right") proprioceptiveDiffVectorFront_.push_back(diff);
    if (proprioceptiveDiffVectorFront_.size() > 12) proprioceptiveDiffVectorFront_.erase(proprioceptiveDiffVectorFront_.begin());
    if (tip == "lefthind" || tip == "righthind") proprioceptiveDiffVectorHind_.push_back(diff);
    if (proprioceptiveDiffVectorHind_.size() > 12) proprioceptiveDiffVectorHind_.erase(proprioceptiveDiffVectorHind_.begin());

    proprioceptiveDiffVector_.push_back(diff);
    if (proprioceptiveDiffVector_.size() > 12) proprioceptiveDiffVector_.erase(proprioceptiveDiffVector_.begin());

    std::vector<double>& varianceDiffVector = proprioceptiveDiffVector_;

    // Calculate Variance.
    double total = 0.0;
    double totalSquared = 0.0;
    for (auto& n : varianceDiffVector){
        total += n;
        totalSquared += pow(n, 2);
    }
    double varianceConsecutiveFootTipPositions = totalSquared / (double)varianceDiffVector.size() - pow(total/(double)varianceDiffVector.size(), 2);
    double mean = total/(double)varianceDiffVector.size();

    std::cout << "varianceConsecutiveFootTipPositions: " << varianceConsecutiveFootTipPositions << std::endl;

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
    setTerrainVariance(varianceConsecutiveFootTipPositions);

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

double SupportSurfaceEstimation::getPenetrationDepthVariance(){
    return penetrationDepthVariance_;
}

double SupportSurfaceEstimation::getDifferentialPenetrationDepthVariance(){
    return differentialPenetrationDepthVariance_;
}

bool SupportSurfaceEstimation::mainGPRegression(double tileResolution, double tileDiameter,
                                                double sideLengthAddingPatch, std::string tip,
                                                const double tipDifference, GridMap& rawMap, GridMap& supportMap, GridMap& fusedMap){

    // Todo: set tile resolution and tile size as config file params
    if (2.0 * tileResolution > tileDiameter) { // TODO: make this double, as using circle iterator
        std::cout << "tile size for gaussian process model tiling must be higher than twice the tile Resolution" << std::endl;
        return false;
    }

    //if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0){

    // Get the difference of the foot tip vs the elevation map to vertically translate the smoothened map..

    std::cout << "tipDifference: " << tipDifference << std::endl;

    // Update sinkage depth Layer.
    //sinkageDepthMapLayerGP(tip, tipDifference);

    // ****************************************************************************************************************************************************************Here i was
    simpleSinkageDepthLayer(tip, tipDifference, supportMap); // Alternative
    //simpleTerrainContinuityLayer(tip, supportMap);

    // Visualization in Rviz.
    visualization_msgs::Marker tileMarkerList;
    tileMarkerList.header.frame_id = "odom_drift_adjusted";
    tileMarkerList.header.stamp = ros::Time();
    tileMarkerList.ns = "elevation_mapping";
    tileMarkerList.id = 0;
    tileMarkerList.type = visualization_msgs::Marker::SPHERE_LIST;
    tileMarkerList.action = visualization_msgs::Marker::ADD;
    tileMarkerList.pose.orientation.x = 0.0;
    tileMarkerList.pose.orientation.y = 0.0;
    tileMarkerList.pose.orientation.z = 0.0;
    tileMarkerList.pose.orientation.w = 1.0;
    tileMarkerList.scale.x = 0.1;
    tileMarkerList.scale.y = 0.1;
    tileMarkerList.scale.z = 0.1;

    tileMarkerList.color.a = 1.0; // Don't forget to set the alpha!
    tileMarkerList.color.r = 1.0;
    tileMarkerList.color.g = 0.1;
    tileMarkerList.color.b = 0.1;


    int noOfTilesPerHalfSide = floor((sideLengthAddingPatch / 2.0) / tileResolution);
    double radius = (sideLengthAddingPatch - 0.15) / 2.0;

    // RQT message publisher.
    geometry_msgs::TwistStamped adaptationMsg;

    // Get uncertainty measures.
    double terrainVariance = getTerrainVariance();
    double supportSurfaceUncertaintyEstimation = getSupportSurfaceUncertaintyEstimation(); //! TODO: uncomment !!!!
    double cumulativeSupportSurfaceUncertaintyEstimation = getCumulativeSupportSurfaceUncertaintyEstimation();
    //std::cout << "CumulativeSupportSurfaceUncertaintyEstimation: " << cumulativeSupportSurfaceUncertaintyEstimation << std::endl;

    // Set lengthscale as function of terrain variance (assessed by foot tips only).
    double factor = 120.0;
    double lengthscale = 5.0 * fmax(5.0 - (terrainVariance * factor), 0.7);
    // Set the weight of the adding procedure as a function of the support surface uncertainty (i.e. difference between foot tip and support surface)
    //double weightFactor = 0.0;
    //if (!isnan(supportSurfaceUncertaintyEstimation)) weightFactor = supportSurfaceUncertaintyEstimation * 0.0001; // Hacked to practically nothing

    double sinkageVariance = getPenetrationDepthVariance();
    double differentialSinkageVariance = getDifferentialPenetrationDepthVariance();

    // TODO: conept for relevance between these two -> two tuning params -> relevance factor AND weightFactor for exp.
    double characteristicValue = (weightTerrainContinuity_ * terrainVariance) / pow((1.0 * sinkageVariance + 0.0 * differentialSinkageVariance), exponentCharacteristicValue_);


    // TODO: reason about this functionality, anything with sqrt?? -> test and learn?


    if (useSignSelectiveContinuityFilter_) {
        // Sign Selective low pass filter.
        if (lowPassFilteredTerrainContinuityValue_ < characteristicValue) lowPassFilteredTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                                * characteristicValue, 0.4), 5.0); // TODO: reason about bounding.
        else lowPassFilteredTerrainContinuityValue_ = characteristicValue;

    }
    else lowPassFilteredTerrainContinuityValue_ = fmin(fmax(continuityFilterGain_ * lowPassFilteredTerrainContinuityValue_ + (1 - continuityFilterGain_)
                                                        * characteristicValue, 0.4), 5.0); // TODO: reason about bounding.



    // Set message values.
    adaptationMsg.header.stamp = ros::Time::now();
    adaptationMsg.twist.linear.x = terrainVariance;
    adaptationMsg.twist.linear.y = lowPassFilteredTerrainContinuityValue_;
    adaptationMsg.twist.angular.x = supportSurfaceUncertaintyEstimation;
    adaptationMsg.twist.angular.y = getMeanGroundTruthDifference();
    adaptationMsg.twist.angular.z = sinkageVariance;
    adaptationMsg.twist.linear.z = differentialSinkageVariance;

    // Get the foot Tip Position.
    Position3 footTip3 = getFootTipPosition3(tip);
    Position footTip(footTip3(0), footTip3(1));

    // Get the foot tip plane coefficients.
    //Eigen::Vector4f planeCoefficients = getFootTipPlaneCoefficients(tip);
    //if (footTipHistoryGP_.size() >= 3) planeCoefficients = getFootTipPlaneCoefficientsFrontOnly(tip); // Testing if it makes a difference.

    for (int i = -noOfTilesPerHalfSide; i <= noOfTilesPerHalfSide; ++i){
        for (int j = -noOfTilesPerHalfSide; j <= noOfTilesPerHalfSide; ++j){
            if (sqrt(pow(i * tileResolution,2) + pow(j * tileResolution,2)) < radius){

                int inputDim = 2;
                int outputDim = 1;
                GaussianProcessRegression<float> myGPR(inputDim, outputDim);
                myGPR.SetHyperParams(GPLengthscale_, GPSigmaN_, GPSigmaF_); // Hacked a factor * 5 remove soon

                Eigen::Vector2f posTile;
                posTile(0) = footTip(0) + i * tileResolution;
                posTile(1) = footTip(1) + j * tileResolution;

                Position posTilePosition(posTile(0), posTile(1));

                // Loop to add training data to GP regression.
                for (CircleIterator iterator(supportMap, posTilePosition, tileDiameter/2.0); !iterator.isPastEnd(); ++iterator) {
                    const Index index(*iterator);
                    Eigen::VectorXf trainInput(inputDim);
                    Eigen::VectorXf trainOutput(outputDim);

                    Position cellPos;
                    supportMap.getPosition(index, cellPos); // This is wrong!!!!!!!!
                    trainInput(0) = cellPos(0);
                    trainInput(1) = cellPos(1);

                    trainOutput(0) = rawMap.at("elevation", index) - supportMap.at("sinkage_depth_gp", index); //! New as part of major changes

                    // SomeTests
                    Eigen::VectorXf trainOutput2(outputDim);
                    trainOutput2(0) = supportMap.at("elevation_gp_added", index);



                    // Probability sampling and replace the training data by plane value
                    // Else do all the following in this loop.

                    bool insert = sampleContinuityPlaneToTrainingData(cellPos, footTip, lowPassFilteredTerrainContinuityValue_);
                    if (insert) {
                        //if (scalarProduct > 0.0) {
                        trainOutput(0) = supportMap.at("terrain_continuity_gp", index);
                        //trainOutput(0) = evaluatePlaneFromCoefficients(planeCoefficients, cellPos);
                        //}
                        // In this case add the old training data, not to influence the past by the continuity assumptions
                        //else {
                        //    trainOutput(0) = supportMap.at("elevation_gp_added", index); // Check, if this is the right way round.
                        //}
                        if (!isnan(trainOutput(0))) myGPR.AddTrainingData(trainInput, trainOutput);
                    }
                    else if (!isnan(trainOutput(0))){  // Removed validation argument here..
                        // Studies for terrain continuity techniques..
                        myGPR.AddTrainingData(trainInput, trainOutput);
                        //if (addFootTipPositionsToGPTraining_){
                        //    myGPR.AddtrainingData()
                        //}


                        // TODO: if continuity assumption: when scalar product is negative: add old data instead of plane data.

                        //if (addOldSupportSurfaceDataToGPTraining_ && i % 3 == 0 && j % 3 == 0){  // Hacked trying some stuff..
                        //    myGPR.AddTrainingData(trainInput, trainOutput2);
                        //}
                    }
                }

                //std::cout << "The Number of Data in this tile is: " << myGPR.get_n_data() << std::endl;

                //myGPR.PrepareRegression(false); // Testing..


                // Only perform regression if no. of Data is sufficiently high.
                if (myGPR.get_n_data() > 1){

                    // Loop here to get test output
                    for (CircleIterator iterator(supportMap, posTilePosition, tileDiameter/2.0); !iterator.isPastEnd(); ++iterator) { // HAcked to raw map for testing..
                        const Index index(*iterator);

                        auto& supportMapElevationGP = supportMap.at("elevation_gp", index);
                        auto& supportMapVarianceGP = supportMap.at("variance_gp", index);

                        //auto& supportMapElevation = supportMap_.at("elevation", index);
                        //auto& supportMapVariance = supportMap_.at("variance", index);
                        //auto& rawMapElevation = rawMap.at("elevation", index);
                        //auto& mapSupSurfaceSmooth = mapSmooth.at("support_surface_smooth", index);


                       // if (!supportMap_.isValid(index)) {
                            //supportMapElevationGP = 0.0;
                        //    supportMapElevation = mapSupSurfaceSmooth; // HAcked for testing..
                        //    supportMapVariance = 0.0; // Hacked -> trust less those layers that are new
                       // }
                      //  else{



                        // Todo, consider hole filling for better hole filling
                        Eigen::VectorXf testInput(inputDim);
                        Eigen::VectorXf testOutput(outputDim);
                        Position inputPosition;
                        supportMap.getPosition(index, inputPosition);

                        testInput(0) = inputPosition(0);
                        testInput(1) = inputPosition(1);

                        double regressionOutput = myGPR.DoRegressionVariance(testInput)(0);
                       // myGPR.DoRegressionVariance(testInput)(0);
                        double regressionOutputVariance = fabs(myGPR.GetVariance()(0));
                        if (isnan(regressionOutputVariance))
                            std::cout << "regressionOutputVariance: ISNAN...." << regressionOutputVariance << std::endl;
                        // Why can this be negative?
                        //std::cout << "regoutVariance :::: ->->->: " << regressionOutputVariance << std::endl;

                        if (!isnan(regressionOutput)){
                            if (!supportMap.isValid(index)){
                                supportMapElevationGP = regressionOutput;
                            }
                            else {
//                                if (supportMapElevationGP == 0.0) std::cout << "ATTENTION!! ZERO BIAS" << std::endl;
                                supportMapElevationGP = 0.5 *  supportMapElevationGP +
                                        (regressionOutput) * 0.5;
                                //supportMapElevationGP = regressionOutput; // Test!!
                                //supportMapElevationGP = myGPR.DoRegression(testInput)(0) + tipDifference; // Hacked to test!
                            }
                        }


//                        if (!isnan(regressionOutputVariance)){
//                            if (!supportMap.isValid(index)){
//                                supportMapVarianceGP = regressionOutputVariance;
//                            }
//                            else {
//                                    supportMapVarianceGP = 0.5 * supportMapVarianceGP +
//                                        (regressionOutputVariance) * 0.5;
//                            }
//                        }
                        //}

                       // if (supportMapElevationGP == 0.0) supportMap_.isValid(index) = false;

                    }

                }

                // Debug:
            //    Eigen::VectorXf testInput(inputDim);
            //    testInput(0) = posTilePosition(0);
            //    testInput(1) = posTilePosition(1);
            //    Eigen::VectorXf testOutput(outputDim);
            //    testOutput(0) =  myGPR.DoRegression(testInput)(0);
                //if (!isnan(testOutput(0))) supportMap_.atPosition("elevation_gp", posTilePosition) = testOutput(0);
                //supportMap_.atPosition("elevation_gp", posTilePosition) = myGPR.DoRegression(testInput)(0);
                // End Debug

                // Visualization:

                geometry_msgs::Point p;
                p.x = posTile(0);
                p.y = posTile(1);
                p.z = 0.0; // Foot tip Plan visualization.. (otherwise set to 0.0)
                tileMarkerList.points.push_back(p);

                supportSurfaceAddingAreaPublisher_.publish(tileMarkerList);
               // myGPR.ClearTrainingData();
            }

        }
    }

    // Weight for tiles against each others.
    double intraGPWeight = 0.8;

    for (CircleIterator iterator(supportMap, footTip, radius); !iterator.isPastEnd(); ++iterator) { // HAcked to raw map for testing..
        const Index index(*iterator);

        Position addingPosition;
        supportMap.getPosition(index, addingPosition);

        double distance = sqrt(pow(addingPosition(0) - footTip(0), 2) +
                               pow(addingPosition(1) - footTip(1), 2));
        double distanceFactor = sqrt(pow(addingPosition(0) - footTip(0), 2) +
                                           pow(addingPosition(1) - footTip(1), 2)) / radius;


        // TODO: here, add only if scalar product is above negative threshold.

        // New weighting scheme here:
        double weight;
        if (distance <= 0.2 * radius) weight = 1.0 - (distance / radius);
        else if (distance >= 0.2 * radius && distance <= weightDecayThreshold_ * radius) weight = 0.8;
        else weight = weightDecayThreshold_ - weightDecayThreshold_ * (distance - weightDecayThreshold_ * radius)
                / ((1.0 - weightDecayThreshold_) * radius);
        // End of new weighting scheme

        auto& supportMapElevationGP = supportMap.at("elevation_gp", index);
        auto& supportMapElevationGPAdded = supportMap.at("elevation_gp_added", index);
        auto& supportMapVarianceGP = supportMap.at("variance_gp", index);
        //auto& supportMapVarianceGPAdded = supportMap.at("variance_gp_added", index);
        //auto& supportMapSinkageDepthGP = supportMap.at("sinkage_depth_gp", index);
        auto& rawMapElevationGP = rawMap.at("elevation_gp_added_raw", index);
        auto& fusedMapElevationGP = fusedMap.at("elevation_gp_added_raw", index);

        if (supportMap.isValid(index) && !isnan(supportMapElevationGP)){
            supportMapElevationGPAdded = (1 - weight) * supportMapElevationGPAdded + (supportMapElevationGP) * weight;
            // Naive approach to variance calculation:
            supportMapVarianceGP = 0.5 * supportMapVarianceGP + 0.5 * pow((supportMapElevationGPAdded - supportMapElevationGP), 2);
        }
        else if (!isnan(supportMapElevationGP)){
            supportMapElevationGPAdded = supportMapElevationGP;
            supportMapVarianceGP = 0.0;
        }
        if (!isnan(supportMapElevationGPAdded)) rawMapElevationGP = supportMapElevationGPAdded; // Test
        if (!isnan(supportMapElevationGPAdded)) fusedMapElevationGP = supportMapElevationGPAdded; // Test


        //if (isnan(supportMap.at("smoothed_top_layer_gp", index))) supportMap.at("smoothed_top_layer_gp", index) = 0.0;

//        if (supportMap.isValid(index) && !isnan(supportMapVarianceGP) && !isnan(supportMapVarianceGPAdded)){
//            supportMapVarianceGPAdded = (1 - weight) * supportMapVarianceGPAdded + (supportMapVarianceGP) * weight;
//        }
//        else if (!isnan(supportMapVarianceGP)){
//            supportMapVarianceGPAdded = supportMapVarianceGP;
//        }
//        else supportMapVarianceGPAdded = 0.0;  // not sound!!!

//        if (isnan(supportMap.at("variance_gp_added", index))) supportMap.at("variance_gp_added", index) = 0.0; // Check if sound!!!!!
//        if (isnan(supportMap.at("variance_gp", index))) supportMap.at("variance_gp", index) = 0.0; // Check if sound!!!!!
    }




    // Publish adaptation Parameters
    varianceTwistPublisher_.publish(adaptationMsg);
    //}

    return true;
}

bool SupportSurfaceEstimation::simpleSinkageDepthLayer(std::string& tip, const double& tipDifference, GridMap& supportMap){

    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0 && leftHindStanceVector_.size() > 0 && rightHindStanceVector_.size() > 0){

        Eigen::Vector3f tipLeftVec = getLatestLeftStance();
        Eigen::Vector3f tipRightVec = getLatestRightStance();
        Position3 tipLeftPos3(tipLeftVec(0), tipLeftVec(1), tipLeftVec(2));
        Position3 tipRightPos3(tipRightVec(0), tipRightVec(1), tipRightVec(2));
        double radius = 0.65;
        int maxSizeFootTipHistory = 15; // See what this does..

        std::cout << "TIP DIFFERENCE::::::::::::::::::::::::::::::::::::::::::::::: " << -tipDifference << std::endl;
        // DEBUG:
        if (-tipDifference < 0.0) std::cout << "Attention, negative tip DIfference found!!: " << -tipDifference << std::endl;
        if (initializedLeftSinkageDepth_ && leftFrontSinkageDepth_ < 0.0) std::cout << "Attention, negative leftfrontsd found!!: " << leftFrontSinkageDepth_ << std::endl;
        if (initializedRightSinkageDepth_ && rightFrontSinkageDepth_ < 0.0) std::cout << "Attention, negative rightfrontsd found!!: " << rightFrontSinkageDepth_ << std::endl;
        if (isnan(leftFrontSinkageDepth_)) std::cout << " NANANANANANANAANNNNNNNNNNNNANNNNNNNAAAAAAAAAAAAAANNNNNNNNNNNNNN" << std::endl;

        // Do history vector.
        Position3 footTip;
        if (tip == "left") footTip = tipLeftPos3;
        if (tip == "right") footTip = tipRightPos3;
        if (tip == "lefthind") footTip = getHindLeftFootTipPosition();
        if (tip == "righthind") footTip = getHindRightFootTipPosition();

        if (isnan(tipDifference)) {
            if (tip == "left" && !isnan(leftFrontSinkageDepth_) && initializedLeftSinkageDepth_) {
                sinkageDepthHistory_.push_back(leftFrontSinkageDepth_); // Do not update the sinkage depth if there is no new information.
                sinkageFootTipHistoryGP_.push_back(footTip);
                if (fabs(leftFrontSinkageDepth_) < 0.001) std::cout << "WARNING: the stored sinkage depth value is very close to zero!!! \n \n \n \n \n WARNING \n";
            }
            if (tip == "right" && !isnan(rightFrontSinkageDepth_) && initializedRightSinkageDepth_) {
                sinkageDepthHistory_.push_back(rightFrontSinkageDepth_);
                sinkageFootTipHistoryGP_.push_back(footTip);
                if (fabs(rightFrontSinkageDepth_) < 0.001) std::cout << "WARNING: the stored sinkage depth value is very close to zero!!! \n \n \n \n \n WARNING \n";
            }
        }
        else {
            sinkageDepthHistory_.push_back(-tipDifference); // Todo nicer with class vars for each foot tip
            sinkageFootTipHistoryGP_.push_back(footTip);
            if (tip == "left"){
                leftFrontSinkageDepth_ = -tipDifference;
                initializedLeftSinkageDepth_ = true;
            }
            if (tip == "right") {
                rightFrontSinkageDepth_ = -tipDifference;
                initializedRightSinkageDepth_ = true;
            }
        }
        if (sinkageFootTipHistoryGP_.size() > maxSizeFootTipHistory) { // They must be the same!!
            sinkageFootTipHistoryGP_.erase(sinkageFootTipHistoryGP_.begin());
            sinkageDepthHistory_.erase(sinkageDepthHistory_.begin());
        }

        if (sinkageFootTipHistoryGP_.size() != sinkageDepthHistory_.size()) {
            std::cout << "Attention, having issues \n \n \n \n ISSUES i said!!" << std::endl;
        }

        Position center(footTip(0), footTip(1));

        for (grid_map::CircleIterator iterator(supportMap, center, radius); !iterator.isPastEnd(); ++iterator) {

            const Index index(*iterator);
            auto& supportMapElevationGPSinkage = supportMap.at("sinkage_depth_gp", index);
            Position pos;
            supportMap.getPosition(index, pos);

            float addingDistance = sqrt(pow(pos(0) - footTip(0), 2) + pow(pos(1) - footTip(1), 2));

            // Keep track of total weight and temporary height value.
            float totalWeight = 0.0;
            float tempHeight = 0.0;

            for (unsigned int i = 0; i < min(sinkageFootTipHistoryGP_.size(), sinkageDepthHistory_.size()); ++i) {
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

            double output;
            if (totalWeight > 0.0001) output = tempHeight / totalWeight;
            else if (!isnan(tipDifference)) output = tipDifference;
            else output = 0.0;

            float addingWeight = fmax(1 - (addingDistance / radius), 0.0);
            if (!isnan(output)){
                if (!supportMap.isValid(index)){
                    supportMapElevationGPSinkage = output;
                }
                else{
                    if (!isnan(supportMapElevationGPSinkage))  // Hacked away, attention!!
                        supportMapElevationGPSinkage = addingWeight * output + (1 - addingWeight) * supportMapElevationGPSinkage;
                }
            }
            //else std::cout << "Output would have been NANANANANANANANANANANA" << std::endl;
            // TODO: idea: exponent of distance proportionality as learning parameter.
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

        double radius = 0.65;
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

                            std::cout << "Crazy triangle area overflow:!! " << triangleArea << std::endl;
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

bool SupportSurfaceEstimation::terrainContinuityLayerGP(std::string& tip, GridMap& supportMap){

    if (leftStanceVector_.size() > 0 && rightStanceVector_.size() > 0){

        Position3 tipPos3;
        if (tip == "left") tipPos3 = getFrontLeftFootTipPosition();
        if (tip == "right") tipPos3 = getFrontRightFootTipPosition();
        if (tip == "lefthind") tipPos3 = getHindLeftFootTipPosition();
        if (tip == "righthind") tipPos3 = getHindRightFootTipPosition();

        Position tipPos(tipPos3(0), tipPos3(1));

        int maxSizeFootTipHistory = 9;

        footTipHistoryGP_.push_back(tipPos3);
        if (footTipHistoryGP_.size() > maxSizeFootTipHistory)
            footTipHistoryGP_.erase(footTipHistoryGP_.begin());

        int inputDim = 2;
        int outputDim = 1;
        float radius = 0.45;

        Eigen::VectorXf trainInput(inputDim);
        Eigen::VectorXf trainOutput(outputDim);

        GaussianProcessRegression<float> continuityGPR(inputDim, outputDim);
        continuityGPR.SetHyperParams(continuityGPLengthscale_, continuityGPSigmaN_, continuityGPSigmaF_);

        for (unsigned int j = 0; j < footTipHistoryGP_.size(); ++j) {

            Position cellPos(footTipHistoryGP_[j](0), footTipHistoryGP_[j](1));
            trainInput(0) = cellPos(0);
            trainInput(1) = cellPos(1);
            trainOutput(0) = footTipHistoryGP_[j](2);

            continuityGPR.AddTrainingData(trainInput, trainOutput);
        }

        for (CircleIterator iterator(supportMap, tipPos, radius); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            Eigen::VectorXf testInput(inputDim);

            auto& supportMapContinuityGP = supportMap.at("terrain_continuity_gp", index);

            Position cellPos;
            supportMap.getPosition(index, cellPos);
            testInput(0) = cellPos(0);
            testInput(1) = cellPos(1);

            double outputHeight = continuityGPR.DoRegression(testInput)(0);  // Hope this shouldnt be testOutput(0)
            double localDistance = sqrt(pow(cellPos(0) - tipPos(0), 2) + pow(cellPos(1) - tipPos(1), 2));

            float addingWeight;

            if (localDistance > 0.0001) addingWeight = fabs(fmax(1.0 - (localDistance / radius), 0.0));
            else addingWeight = 0.0;

            //std::cout << "adding weight : " << addingWeight << std::endl;

            if (!isnan(outputHeight)){
                //if (true) {
                if (isnan(supportMapContinuityGP) || !supportMap.isValid(index)){
                    supportMapContinuityGP = outputHeight;
                }
                else{
                    if (!isnan(supportMapContinuityGP))  // Hacked away, attention!!
                        supportMapContinuityGP = addingWeight * outputHeight + (1.0 - addingWeight) * supportMapContinuityGP;
                }
            }
        }
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
    Matrix& dataSup = supportSurfaceMap["elevation_gp_added"];
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
        auto& supportMapElevation = supportMap.at("elevation_gp_added", index);
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
        std::cout << "global cell counter: " << overallConsideredStanceCounter_ << std::endl;
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

    Difference = fabs(supportMap.atPosition("elevation_gp_added", cellPosition) - heightGroundTruth);

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

} /* namespace */
