/*
 * StructuredLightSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <limits>
#include <string>

// New **********************************************************
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/ros/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
// End New *********************************************************

namespace elevation_mapping {

/*! StructuredLight-type (structured light) sensor model:
 * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance - sensorModelNormalFactorC_)^2;
 * standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance
 * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
 */

StructuredLightSensorProcessor::StructuredLightSensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{
    // New **************************************************************************
    //spatialVarianceColoreSchemedPointcloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("spatial_variance_pointcloud", 1);
    // End New *******************************************************************************
}

StructuredLightSensorProcessor::~StructuredLightSensorProcessor()
{

}

bool StructuredLightSensorProcessor::readParameters()
{
  SensorProcessorBase::readParameters();
  nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], std::numeric_limits<double>::min());
  nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], std::numeric_limits<double>::max());
  nodeHandle_.param("sensor_processor/normal_factor_a", sensorParameters_["normal_factor_a"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_b", sensorParameters_["normal_factor_b"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_c", sensorParameters_["normal_factor_c"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_d", sensorParameters_["normal_factor_d"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_e", sensorParameters_["normal_factor_e"], 0.0);
  nodeHandle_.param("sensor_processor/lateral_factor", sensorParameters_["lateral_factor"], 0.0);
  return true;
}

bool StructuredLightSensorProcessor::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
	pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

	passThroughFilter.setInputCloud(pointCloud);
	passThroughFilter.setFilterFieldName("z");
	passThroughFilter.setFilterLimits(sensorParameters_.at("cutoff_min_depth"), sensorParameters_.at("cutoff_max_depth"));
	// This makes the point cloud also dense (no NaN points).
	passThroughFilter.filter(tempPointCloud);
	tempPointCloud.is_dense = true;
	pointCloud->swap(tempPointCloud);

	ROS_DEBUG("cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
	return true;
}

bool StructuredLightSensorProcessor::computeVariances(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		Eigen::VectorXf& variances)
{
	variances.resize(pointCloud->size());

	// Projection vector (P).
	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

	// Sensor Jacobian (J_s).
	const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

	// Robot rotation covariance matrix (Sigma_q).
	Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));


  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
		// For every point in point cloud.

		// Preparation.
		auto& point = pointCloud->points[i];
		Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
		float heightVariance = 0.0; // sigma_p

		// Measurement distance.
        float measurementDistance = pointVector.z();


		// Compute sensor covariance matrix (Sigma_S) with sensor model.
                float deviationNormal = sensorParameters_.at("normal_factor_a")
                    + sensorParameters_.at("normal_factor_b")
                        * (measurementDistance - sensorParameters_.at("normal_factor_c")) * (measurementDistance - sensorParameters_.at("normal_factor_c"))
                    + sensorParameters_.at("normal_factor_d") * pow(measurementDistance, sensorParameters_.at("normal_factor_e"));
		float varianceNormal = deviationNormal * deviationNormal;
		float deviationLateral = sensorParameters_.at("lateral_factor") * measurementDistance;
		float varianceLateral = deviationLateral * deviationLateral;
		Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
		sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

		// Robot rotation Jacobian (J_q).
		const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
		Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

		// Measurement variance for map (error propagation law).
		heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
		heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

		// Copy to list.
		variances(i) = heightVariance;
	}


	return true;
}

// New ***************************************************************************************************************
bool StructuredLightSensorProcessor::computeSpatialVariances(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapFrame,
        Eigen::VectorXf& spatialVariances)
{

    spatialVariances.resize(pointCloudMapFrame->size());

    // Parameters.
    int proximityParameter = 1500;
    double thresholdRadius = 0.01;

    for (unsigned int i = 0; i < pointCloudMapFrame->size(); ++i) {
        // For every point in point cloud.

        // Preparation.
        auto& point = pointCloudMapFrame->points[i];
        float varianceAccumulator = 0;
        float varianceAccumulatorSquared = 0;
        int varianceCalculationPointCounter = 0;


        for (unsigned int j = std::max(0, int(i-proximityParameter)); j < std::min(int(pointCloudMapFrame->size()), int(i+proximityParameter)); ++j){
            // For every point in logical proximity of point[j] (Pointcloud indexing).

            auto& pointIter = pointCloudMapFrame->points[j];

            // If the point is within the thresholdRadius of point[j].
            if(sqrt(pow((point.x-pointIter.x),2)+pow((point.y-pointIter.y),2)) < thresholdRadius) {
                varianceAccumulator += std::abs(pointIter.z);
                varianceAccumulatorSquared += pow(pointIter.z,2);
                varianceCalculationPointCounter++;
            }
        }

        // Assign spatialVariance (Prevent division by zero).
        float spatialVariance;
        if(varianceCalculationPointCounter < 0.1){
            std::cout << "WARNING: 0 points" << std::endl;
            spatialVariance = 0;
        }
        else{
            spatialVariance = std::abs(varianceAccumulatorSquared / float(varianceCalculationPointCounter)
                                       - pow(varianceAccumulator / float(varianceCalculationPointCounter),2));

        }

        // For Debugging.
        if(i%1000 == 0){
            std::cout << "Variances: " << spatialVariance << " X: "<< point.x << " Y: " << point.y << " Z: " << point.z << "P" << std::endl;
        }

        // Copy to List.
        spatialVariances[i] = spatialVariance;


        //pcl::PointXYZRGB pointOut = point;
        //pointOut.r = 255;
        //PointCloudColorSchemeSpatialVariance.push_back(pointOut);
        // std::cout << "RGB: " << "R: " << point.r << " G:" << point.g << " B: " << point.b << std::endl;

        // End New *****************************************************




    }
    //spatialVarianceColoreSchemedPointcloudPublisher_.publish()
    //sensor_msgs::PointCloud2 OutputVariancePointcloud;
    //OutputVariancePointcloud.header.frame_id = "/map";

    //pcl::toROSMsg(PointCloudColorSchemeSpatialVariance, OutputVariancePointcloud);
    //spatialVarianceColoreSchemedPointcloudPublisher_.publish(OutputVariancePointcloud);

    // New
    //meanSpatialVariance = meanSpatialVariance / (float)pointCloudMapFrame->size();
    //std::cout << "Mean Spatial Variance: " << meanSpatialVariance << std::endl;
    // End New
    return true;
    // End New ***********************************************************************************************************
}

} /* namespace */
