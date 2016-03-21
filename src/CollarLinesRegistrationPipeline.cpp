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

#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/PoseGraphEdge.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace velodyne_pointcloud;

namespace but_velodyne
{

Eigen::Matrix4f CollarLinesRegistrationPipeline::registerTwoGrids(const PolarGridOfClouds &source,
                                               const PolarGridOfClouds &target,
                                               const Eigen::Matrix4f &initial_transformation,
                                               int &iterations,
                                               float &error) {
  iterations = 0;
  Termination termination(pipeline_params.minIterations, pipeline_params.maxIterations, pipeline_params.maxTimeSpent,
                          pipeline_params.significantErrorDeviation, pipeline_params.targetError);
  Eigen::Matrix4f transformation = initial_transformation;
  while(!termination()) {

    LineCloud source_line_cloud(source, pipeline_params.linesPerCellGenerated,
                                pipeline_params.linesPerCellPreserved, pipeline_params.preservedFactorOfLinesBy);
    LineCloud target_line_cloud(target, pipeline_params.linesPerCellGenerated,
                                pipeline_params.linesPerCellPreserved, pipeline_params.preservedFactorOfLinesBy);

    CollarLinesRegistration icl_fitting(source_line_cloud, target_line_cloud,
                                        registration_params, transformation);
    for(int sampling_it = 0;
        (sampling_it < pipeline_params.iterationsPerSampling) && !termination();
        sampling_it++) {
      error = icl_fitting.refine();
      termination.addNewError(error);
      iterations++;
    }
    transformation = icl_fitting.getTransformation();
  }
  return transformation;
}

vector<Eigen::Matrix4f> CollarLinesRegistrationPipeline::runRegistrationEffective(
    const PolarGridOfClouds::Ptr &target_grid_cloud) {

  assert(!history.empty());
  stopwatch.restart();

  Eigen::Matrix4f transformation = getPrediction();
  vector<Eigen::Matrix4f> results;
  int iterations = 0;
  for(int i = 0; i < history.size(); i++) {
    int current_iterations;
    float error;
    transformation = registerTwoGrids(*(history[i].getGridCloud()), *target_grid_cloud,
                                      transformation, current_iterations, error);
    iterations += current_iterations;
    printInfo(stopwatch.elapsed(), iterations, transformation, error);
    results.push_back(transformation);

    Eigen::Matrix4f edge_trasform = history[i].getTransformation() * transformation;
    PoseGraphEdge edge(history[i].getIndex(), pose_index, edge_trasform);
    graph_file << edge << endl;
  }

  return results;
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::pickBestByError(const PolarGridOfClouds::Ptr target_cloud,
                                              const vector<Eigen::Matrix4f> &transformations) {
  float best_error = INFINITY;
  Eigen::Matrix4f best_transform;
  LineCloud source(*(history.front().getGridCloud()), pipeline_params.linesPerCellGenerated,
                   pipeline_params.linesPerCellPreserved, pipeline_params.preservedFactorOfLinesBy);
  LineCloud target(*target_cloud, pipeline_params.linesPerCellGenerated,
                   pipeline_params.linesPerCellPreserved, pipeline_params.preservedFactorOfLinesBy);
  int i = 0;
  int best_index = -1;
  for(vector<Eigen::Matrix4f>::const_iterator t = transformations.begin();
      t < transformations.end(); t++, i++) {
    CollarLinesRegistration icl_fitting(source, target, registration_params, *t);
    float error = icl_fitting.computeError();
    if(error < best_error) {
      best_error = error;
      best_transform = *t;
      best_index = i;
    }
    BUT_VELODYNE_LOG << "error of " << i << "th item: " << error << endl << flush;
  }
  BUT_VELODYNE_LOG << "picked: " << best_index << "th" << endl << flush;
  return best_transform;
}

void CollarLinesRegistrationPipeline::pickBestByAverage(const vector<Eigen::Matrix4f> &transformations,
                         Eigen::Matrix4f &mean_transformation,
                         cv::Mat &covariance) {
  vector<MoveParameters> meassurements;
  for(vector<Eigen::Matrix4f>::const_iterator t = transformations.begin();
        t < transformations.end(); t++) {
    meassurements.push_back(MoveParameters(*t));
  }

  MoveParameters mean(0, 0, 0, 0, 0, 0);
  MoveParameters::getGaussDistributionOf(meassurements, mean, covariance);

  mean_transformation =  getTransformation(mean.x, mean.y, mean.z,
                                           mean.roll, mean.pitch, mean.yaw).matrix();
}

void CollarLinesRegistrationPipeline::updateHistory(const PolarGridOfClouds::Ptr target_polar_grid,
                                 Eigen::Matrix4f transformation) {
  for(int i = 0; i < history.size(); i++) {
    history[i].update(transformation);
  }
  history.push_front(HistoryRecord(target_polar_grid, pose_index));
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::runRegistration(const VelodynePointCloud &target_cloud,
                                              Mat &covariance) {
  PolarGridOfClouds::Ptr target_polar_grid = PolarGridOfClouds::of(target_cloud);
  Eigen::Matrix4f transformation;
  if(!history.empty()) {
    pickBestByAverage(runRegistrationEffective(target_polar_grid),
                      transformation, covariance);
    estimation.addMeassurement(transformation);
  } else {
    transformation = Eigen::Matrix4f::Identity();
    covariance = cv::Mat::zeros(6, 6, CV_64FC1);
  }
  updateHistory(target_polar_grid, transformation);
  pose_index++;
  return transformation;
}

void CollarLinesRegistrationPipeline::output(const Eigen::Matrix4f &transformation) {
  cumulated_transformation = cumulated_transformation * transformation;

  Eigen::Matrix4f::Scalar *pose = cumulated_transformation.data();
  std::cout << pose[0] << " " << pose[4] << " " << pose[8]  << " " << pose[12] <<
        " " << pose[1] << " " << pose[5] << " " << pose[9]  << " " << pose[13] <<
        " " << pose[2] << " " << pose[6] << " " << pose[10] << " " << pose[14] << std::endl;
}

Eigen::Matrix4f CollarLinesRegistrationPipeline::getPrediction() {
  Eigen::Matrix4f prediction = estimation.predict();
  BUT_VELODYNE_LOG << "Prediction:" << std::endl << prediction << std::endl << std::endl;
  return prediction;
}

void CollarLinesRegistrationPipeline::printInfo(float time, int iterations, Eigen::Matrix4f t, float error) {
  BUT_VELODYNE_LOG << std::endl <<
      "TOTAL" << std::endl <<
      " * time: " << time << "s" << std::endl <<
      " * iterations: " << iterations << std::endl <<
      " * error: " << error << std::endl <<
      "Transformation:" << std::endl << t << std::endl <<
      "*******************************************************************************" <<
      std::endl << std::flush;
}

} /* namespace but_velodyne */
