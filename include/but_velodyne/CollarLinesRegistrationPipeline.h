/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include <cstdlib>
#include <cstdio>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/point_types.h>
#include <boost/circular_buffer.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/PointCloudLine.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/MoveEstimation.h>
#include <but_velodyne/Termination.h>

#include <iostream>
#define BUT_VELODYNE_LOG std::cerr << getpid() << ": "

namespace but_velodyne
{

/**!
 * Previously registered Velodyne frame. The position of each 3D point
 * with respect to the current sensor position is kept.
 */
class HistoryRecord {
public:

  /**!
   * @param grid_cloud LiDAR measurement in polar grid structure
   * @param index identifier of measurement
   */
  HistoryRecord(PolarGridOfClouds::Ptr grid_cloud, const int index) :
    grid_cloud(grid_cloud), transformation(Eigen::Matrix4f::Identity()),
    index(index) {
  }

  /**!
   * Update the position of points when the sensor is moved.
   *
   * @param t_update change of the sensor position
   */
  void update(const Eigen::Matrix4f &t_update) {
    Eigen::Matrix4f inverse = t_update.inverse();
    grid_cloud->transform(inverse);
    transformation *= t_update;
  }

  /**!
   * @returns points in polar grid structure.
   */
  const PolarGridOfClouds::Ptr& getGridCloud() const
  {
    return grid_cloud;
  }

  /**!
   * @return transformation from previous sensor position (when the measurement was taken)
   * to the current sensor position.
   */
  const Eigen::Matrix4f& getTransformation() const
  {
    return transformation;
  }

  /**!
   * @return identifier
   */
  int getIndex() const
  {
    return index;
  }

private:
  PolarGridOfClouds::Ptr grid_cloud;
  Eigen::Matrix4f transformation;
  int index;
};

/**!
 * Registration of multiple Velodyne LiDAR point clouds (sequence).
 */
class CollarLinesRegistrationPipeline {
public:

  /**!
   * Parameters of registration pipeline
   */
  class Parameters {
  public:
    Parameters(
        int linesPerCellGenerated_ = 20,
        int linesPerCellPreserved_ = 5,
        LineCloud::PreservedFactorBy preservedFactorOfLinesBy_ = LineCloud::NONE,
        int minIterations_ = 20,
        int maxIterations_ = 500,
        int maxTimeSpent_ = 20,  // sec
        int iterationsPerSampling_ = 1000,       // (iterationsPerSampling > maxIterations) causes no resampling
        float targetError_ = 0.01,
        float significantErrorDeviation_ = 0.00001,
        int historySize_ = 1)
    :
      linesPerCellGenerated(linesPerCellGenerated_),
      linesPerCellPreserved(linesPerCellPreserved_),
      preservedFactorOfLinesBy(preservedFactorOfLinesBy_),
      minIterations(minIterations_),
      maxIterations(maxIterations_),
      maxTimeSpent(maxTimeSpent_),
      iterationsPerSampling(iterationsPerSampling_),
      targetError(targetError_),
      significantErrorDeviation(significantErrorDeviation_),
      historySize(historySize_) {
    }

    int linesPerCellGenerated;  /// how many collar lines are generated per polar bin
    int linesPerCellPreserved;  /// how many collar lines are preserved within each bin
    LineCloud::PreservedFactorBy preservedFactorOfLinesBy;      /// [Experimental] how is the probability of collar line estimated

    int minIterations;                  /// minimal number of algorithm iterations
    int maxIterations;                  /// algorithm is terminated after maxIterations is reached
    int maxTimeSpent;                   /// algorithm is terminated after maxTimeSpent seconds
    int iterationsPerSampling;          /// point cloud is re-sampled by collar lines after N iterations
    float targetError;                  /// algorithm is terminated when error is smaller
    float significantErrorDeviation;    /// algorithm is terminated when standard deviation of error is smaller

    int historySize;                    /// number of previous frames used for multi-view approach

  } pipeline_params;

  /**!
   * @param estimator odometry predictor - Kalman filter or Linear predictor
   * @param graph_file destination of pose graph (in SLAM++ format)
   * @param pipeline_params_ parameters of registration pipeline
   * @param registration_params_ parameters of collar line clouds registration itself
   */
  CollarLinesRegistrationPipeline(MoveEstimator &estimator, ostream &graph_file,
                                  Parameters pipeline_params_, CollarLinesRegistration::Parameters registration_params_) :
    estimation(estimator), cumulated_transformation(Eigen::Matrix4f::Identity()),
    pose_index(0), graph_file(graph_file), history(pipeline_params_.historySize),
    pipeline_params(pipeline_params_), registration_params(registration_params_) {
  }

  /**!
   * Registration of new LiDAR meassurement.
   *
   * @param target_cloud new LiDAR meassurement
   * @param covariance [output] covariance matrix of registration for SLAM++ (how much the algorithm trusts the result)
   */
  Eigen::Matrix4f runRegistration(const VelodynePointCloud &target_cloud,
                                  cv::Mat &covariance);

  /**!
   * Prints out the estimation of current sensor pose in KITTI format
   *
   * @param odometry estimation between times when last and current frame were taken
   */
  void output(const Eigen::Matrix4f &transformation);

protected:

  Eigen::Matrix4f getPrediction();

  std::vector<Eigen::Matrix4f> runRegistrationEffective(
      const PolarGridOfClouds::Ptr &target_grid_cloud);

  void printInfo(float time, int iterations, Eigen::Matrix4f t, float error);

  Eigen::Matrix4f registerTwoGrids(const PolarGridOfClouds &source,
                                   const PolarGridOfClouds &target,
                                   const Eigen::Matrix4f &initial_transformation,
                                   int &iterations,
                                   float &error);

  void updateHistory(const PolarGridOfClouds::Ptr target_polar_grid,
                     Eigen::Matrix4f transformation);

  Eigen::Matrix4f pickBestByError(const PolarGridOfClouds::Ptr target_cloud,
                                  const vector<Eigen::Matrix4f> &transformations);

  void pickBestByAverage(const vector<Eigen::Matrix4f> &transformations,
                         Eigen::Matrix4f &mean_transformation,
                         cv::Mat &covariance);

private:
  MoveEstimation estimation;
  Eigen::Matrix4f cumulated_transformation;
  Stopwatch stopwatch;

  boost::circular_buffer<HistoryRecord> history;

  int pose_index;
  ostream &graph_file;

  CollarLinesRegistration::Parameters registration_params;
};

} /* namespace but_velodyne */

#endif /* REGISTRATION_H_ */
