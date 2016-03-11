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

#include <but_velodyne/LineCloud.h>
#include <but_velodyne/Visualizer3D.h>

namespace but_velodyne
{
/**!
 * Registration of two collar line clouds.
 */
class CollarLinesRegistration
{
  typedef Eigen::Vector3f TPoint3D;
  typedef Eigen::Matrix<TPoint3D::Scalar, TPoint3D::RowsAtCompileTime, Eigen::Dynamic> MatrixOfPoints;
  typedef Eigen::DiagonalMatrix<TPoint3D::Scalar, Eigen::Dynamic, Eigen::Dynamic> WeightsMatrix;
public:

  /**!
   * How the weights are assigned to the collar line matches.
   */
  enum Weights {
    DISTANCE_WEIGHTS,           // matches of close lines are more significant
    VERTICAL_ANGLE_WEIGHTS,     // matches of vertical lines are more significant
    NO_WEIGHTS                  // all line matches are equal
  };

  friend std::istream& operator>> (std::istream &in, Weights &weightning);

  /**!
   * How the threshold for line matches filtering is estimated.
   */
  enum Threshold {
      MEDIAN_THRESHOLD,         // all matches with distance above median are discarded
      MEAN_THRESHOLD,           // threshold = mean
      NO_THRESHOLD              // no thresholding - all matches are preserved
  };

  friend std::istream& operator>> (std::istream &in, Threshold &thresholding);

  /**!
   * Options of registration.
   */
  class Parameters
  {
  public:
    Parameters(
        Threshold distance_threshold_ = MEDIAN_THRESHOLD,
        Weights weighting_ = NO_WEIGHTS,
        int correnspPerLineMatch_ = 1,
        float lineCorrenspSigma_ = 0.0001) :
        distance_threshold(distance_threshold_),
        weighting(weighting_),
        correnspPerLineMatch(correnspPerLineMatch_),
        lineCorrenspSigma(lineCorrenspSigma_){
    }
    Threshold distance_threshold;       /// how is the threshold of line matches distance estimated
    Weights weighting;                  /// optional weighting of line matches
    int correnspPerLineMatch;           /// [Experimental] how many corresponding points are generated per line match
    float lineCorrenspSigma;            /// [Experimental] deviation of Gaussian noise added to the point correspondences
  } params;

  /**!
   * @param source_cloud_ the line cloud from time T
   * @param target_cloud_ the line cloud from time T+1
   * @param params_ parameters of registration
   * @param initial_transformation_ predicted transformation for initialization
   */
  CollarLinesRegistration(const LineCloud &source_cloud_,
                            const LineCloud &target_cloud_,
                            const Parameters params_,
                            const Eigen::Matrix4f initial_transformation_ = Eigen::Matrix4f::Identity()) :
    source_cloud(source_cloud_), target_cloud(target_cloud_),
    initial_transformation(initial_transformation_),
    params(params_),
    transformation(Eigen::Matrix4f::Identity()),
    matching_time(0), correnspondences_time(0), tranformation_time(0), error_time(0) {

    source_kdtree.setInputCloud(source_cloud.line_middles.makeShared());
    this->target_cloud.transform(initial_transformation);
  }

  /**!
   * Run iteration of the registration process.
   *
   * @return average distance (error) of the matching lines
   */
  float refine();

  /**!
   * @return matches of collar lines between source (train indices) and target (queries) cloud
   */
  std::vector<cv::DMatch> getMatches() {
    return matches;
  }

  /**!
   * @return transformation estimated so far
   */
  const Eigen::Matrix4f getTransformation() const
  {
    return initial_transformation * transformation;
  }

  /**!
   * Visualize line correspondences found in last iteration
   */
  void showLinesCorrenspondences();

  /**!
   * @return error (average distance of matching lines) of the last iteration
   */
  float computeError();

  // time counters measuring how much the each step of registration process costs
  float matching_time, correnspondences_time, tranformation_time, error_time;

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
};

} /* namespace but_velodyne */

#endif /* ITERATIVELINEPLANEFITTING_H_ */
