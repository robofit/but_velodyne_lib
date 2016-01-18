/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2015
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

#ifndef ITERATIVELINEPLANEFITTING_H_
#define ITERATIVELINEPLANEFITTING_H_

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <but_velodyne_odom/LineCloud.h>
#include <but_velodyne_odom/Visualizer3D.h>

namespace but_velodyne_odom
{

class CollarLinesRegistration
{
  typedef Eigen::Vector3f TPoint3D;
  typedef Eigen::Matrix<TPoint3D::Scalar, TPoint3D::RowsAtCompileTime, Eigen::Dynamic> MatrixOfPoints;
  typedef Eigen::DiagonalMatrix<TPoint3D::Scalar, Eigen::Dynamic, Eigen::Dynamic> WeightsMatrix;
public:
  enum Weights {
    DISTANCE_WEIGHTS,
    VERTICAL_ANGLE_WEIGHTS,
    NO_WEIGHTS
  };

  CollarLinesRegistration(const LineCloud &source_cloud,
                            const LineCloud &target_cloud,
                            const int distancce_threshold,
                            const enum Weights weighting,
                            const Eigen::Matrix4f initial_transformation = Eigen::Matrix4f::Identity()) :
    source_cloud(source_cloud), target_cloud(target_cloud),
    initial_transformation(initial_transformation),
    transformation(Eigen::Matrix4f::Identity()),
    distance_threshold(distancce_threshold), weighting(weighting),
    matching_time(0), correnspondences_time(0), tranformation_time(0), error_time(0) {

    source_kdtree.setInputCloud(source_cloud.line_middles.makeShared());
    this->target_cloud.transform(initial_transformation);
  }

  float refine();

  std::vector<cv::DMatch> getMatches() {
    return matches;
  }

  const Eigen::Matrix4f getTransformation() const
  {
    //std::cerr << "refined_transf" << std::endl << transformation << std::endl;
    //std::cerr << "initial_transf" << std::endl << initial_transformation << std::endl;
    return initial_transformation * transformation;
  }

  void showLinesCorrenspondences();

  float computeError();

  float matching_time, correnspondences_time, tranformation_time, error_time;

  static const int MEDIAN_THRESHOLD = -1;
  static const int MEAN_THRESHOLD = -2;
  static const int NO_THRESHOLD = -3;

protected:
  void findClosestMatchesByMiddles();

  void getCorrespondingPoints(MatrixOfPoints &source_coresp_points,
                              MatrixOfPoints &target_coresp_points);

  void getWeightingMatrix(WeightsMatrix &weightingMatrix);

  Eigen::Matrix4f computeTransformationWeighted(const MatrixOfPoints &source_coresp_points,
                             const MatrixOfPoints &target_coresp_points);

  float computeError(const MatrixOfPoints &source_coresp_points,
                     const MatrixOfPoints &target_coresp_points,
                     const Eigen::Matrix4f &transformation);

  void filterMatchesByThreshold(const float threshold);

  float getVerticalWeight(const Eigen::Vector3f &source_line_orient,
                          const Eigen::Vector3f &target_line_orient);

  float sinOfAngleWithGround(const Eigen::Vector3f &orientation);

  float getMatchesMedian();
  float getMatchesMean();

private:
  const LineCloud &source_cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> source_kdtree;
  LineCloud target_cloud;
  std::vector<cv::DMatch> matches;
  std::vector<cv::DMatch> rejected_matches;
  Eigen::Matrix4f initial_transformation;
  Eigen::Matrix4f transformation;
  Eigen::VectorXf correspondences_weights;
  float const distance_threshold;
  const Weights weighting;

  static const int correnspPerLineMatch = 10;
  static const float lineCorrenspSigma = 0.1;        // mostly within 10cm
};

} /* namespace but_velodyne_odom */

#endif /* ITERATIVELINEPLANEFITTING_H_ */
