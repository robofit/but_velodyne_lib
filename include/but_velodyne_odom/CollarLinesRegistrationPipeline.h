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

#include <but_velodyne_odom/VelodynePointCloud.h>
#include <but_velodyne_odom/Stopwatch.h>
#include <but_velodyne_odom/PointCloudLine.h>
#include <but_velodyne_odom/Visualizer3D.h>
#include <but_velodyne_odom/PolarGridOfClouds.h>
#include <but_velodyne_odom/LineCloud.h>
#include <but_velodyne_odom/CollarLinesRegistration.h>
#include <but_velodyne_odom/MoveEstimation.h>
#include <but_velodyne_odom/Termination.h>

#define log cerr << getpid() << ": "

namespace but_velodyne_odom
{

class HistoryRecord {
public:
  HistoryRecord(PolarGridOfClouds::Ptr grid_cloud, const int index) :
    grid_cloud(grid_cloud), transformation(Eigen::Matrix4f::Identity()),
    index(index) {
  }

  void update(const Eigen::Matrix4f &t_update) {
    Eigen::Matrix4f inverse = t_update.inverse();
    grid_cloud->transform(inverse);
    transformation *= t_update;
  }

  const PolarGridOfClouds::Ptr& getGridCloud() const
  {
    return grid_cloud;
  }

  const Eigen::Matrix4f& getTransformation() const
  {
    return transformation;
  }

  int getIndex() const
  {
    return index;
  }

private:
  PolarGridOfClouds::Ptr grid_cloud;
  Eigen::Matrix4f transformation;
  int index;
};

class CollarLinesRegistrationPipeline {
public:
  CollarLinesRegistrationPipeline(MoveEstimator &estimator, ostream &graph_file) :
    estimation(estimator), cumulated_transformation(Eigen::Matrix4f::Identity()),
    pose_index(0), graph_file(graph_file), history(HISTORY_SIZE),
    weights_by(CollarLinesRegistration::NO_WEIGHTS),
    linesPerCellGenerated(DEFAULT_CELL_LINES_GENERATED),
    linesPerCellPreserved(DEFAULT_CELL_LINES_PRESERVED),
    maxIterations(DEFAULT_MAX_ITERATIONS),
    maxTimeSpent(DEFAULT_MAX_TIME_SPENT),
    iterationsPerSampling(DEFAULT_ITERATIONS_PER_SAMPLING),
    minIterations(DEFAULT_MIN_ITERATIONS) {
  }

  Eigen::Matrix4f runRegistration(const VelodynePointCloud &target_cloud,
                                  cv::Mat &covariance);

  void output(const Eigen::Matrix4f &transformation) {
    cumulated_transformation = cumulated_transformation * transformation;

    Eigen::Matrix4f::Scalar *pose = cumulated_transformation.data();
    std::cout << pose[0] << " " << pose[4] << " " << pose[8]  << " " << pose[12] <<
          " " << pose[1] << " " << pose[5] << " " << pose[9]  << " " << pose[13] <<
          " " << pose[2] << " " << pose[6] << " " << pose[10] << " " << pose[14] << std::endl;
  }

  void setWeightsBy(CollarLinesRegistration::Weights weightsBy)
  {
    weights_by = weightsBy;
  }

  void setLinesPerCellGenerated(const int linesPerCellGenerated)
  {
    this->linesPerCellGenerated = linesPerCellGenerated;
  }

  void setLinesPerCellPreserved(const int linesPerCellPreserved)
  {
    this->linesPerCellPreserved = linesPerCellPreserved;
  }

  void setMaxIterations(int maxIterations)
  {
    this->maxIterations = maxIterations;
  }

  void setMaxTimeSpent(int maxTimeSpent)
  {
    this->maxTimeSpent = maxTimeSpent;
  }

  void setIterationsPerSampling(int iterationsPerSampling)
  {
    this->iterationsPerSampling = iterationsPerSampling;
  }

  void setMinIterations(int minIterations)
  {
    this->minIterations = minIterations;
  }

protected:

  Eigen::Matrix4f getPrediction() {
    Eigen::Matrix4f prediction = estimation.predict();
    log << "Prediction:" << std::endl << prediction << std::endl << std::endl;
    return prediction;
  }

  std::vector<Eigen::Matrix4f> runRegistrationEffective(
      const PolarGridOfClouds::Ptr &target_grid_cloud);

  void printInfo(float time, int iterations, Eigen::Matrix4f t, float error) {
    log << std::endl <<
        "TOTAL" << std::endl <<
        " * time: " << time << "s" << std::endl <<
        " * iterations: " << iterations << std::endl <<
        " * error: " << error << std::endl <<
        "Transformation:" << std::endl << t << std::endl <<
        "*******************************************************************************" <<
        std::endl << std::flush;
  }

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
  static const float TARGET_ERROR = 0.01;
  static const float DEFAULT_MAX_TIME_SPENT = 20;        // sec
  static const float SIGNIFICANT_DEVIATION = 0.00001;
  static const int DEFAULT_MIN_ITERATIONS = 20;
  static const int DEFAULT_MAX_ITERATIONS = 500;
  static const int DEFAULT_ITERATIONS_PER_SAMPLING = 1000;

  static const int DEFAULT_CELL_LINES_GENERATED = 20;
  static const int DEFAULT_CELL_LINES_PRESERVED = 5;

  static const int HISTORY_SIZE = 1;

  CollarLinesRegistration::Weights weights_by;

  int linesPerCellGenerated;
  int linesPerCellPreserved;

  int maxIterations;
  int minIterations;
  int maxTimeSpent;
  int iterationsPerSampling;

  MoveEstimation estimation;
  Eigen::Matrix4f cumulated_transformation;
  Stopwatch stopwatch;

  boost::circular_buffer<HistoryRecord> history;

  int pose_index;
  ostream &graph_file;

public:
  static KalmanMoveEstimator kalman_estimator;
  static LinearMoveEstimator linear_estimator;
};

} /* namespace but_velodyne_odom */

#endif /* REGISTRATION_H_ */
