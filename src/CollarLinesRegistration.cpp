/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2014
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

#include <numeric>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/program_options/errors.hpp>

#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/EigenUtils.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace pcl;
using namespace boost;

namespace but_velodyne
{

std::istream& operator>> (std::istream &in, CollarLinesRegistration::Weights &weightning) {
  string token;
  in >> token;

  boost::to_upper(token);
  if (token == "DISTANCE_WEIGHTS") {
    weightning = CollarLinesRegistration::DISTANCE_WEIGHTS;
  } else if (token == "VERTICAL_ANGLE_WEIGHTS") {
    weightning = CollarLinesRegistration::VERTICAL_ANGLE_WEIGHTS;
  } else if (token == "NO_WEIGHTS") {
    weightning = CollarLinesRegistration::NO_WEIGHTS;
  } else {
      throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value,
                                                     "lines_preserved_factor_by");
  }
  return in;
}

std::istream& operator>> (std::istream &in, CollarLinesRegistration::Threshold &thresholding) {
  string token;
  in >> token;

  boost::to_upper(token);
  if (token == "MEDIAN_THRESHOLD") {
    thresholding = CollarLinesRegistration::MEDIAN_THRESHOLD;
  } else if (token == "MEAN_THRESHOLD") {
    thresholding = CollarLinesRegistration::MEAN_THRESHOLD;
  } else if (token == "NO_THRESHOLD") {
    thresholding = CollarLinesRegistration::NO_THRESHOLD;
  } else {
      throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value,
                                                     "lines_preserved_factor_by");
  }
  return in;
}


float CollarLinesRegistration::refine() {
  Stopwatch watch;

  watch.restart();
  findClosestMatchesByMiddles();
  matching_time += watch.elapsed();

  watch.restart();
  MatrixOfPoints source_coresp_points(size_t(TPoint3D::RowsAtCompileTime), matches.size()*params.correnspPerLineMatch);
  MatrixOfPoints target_coresp_points(size_t(TPoint3D::RowsAtCompileTime), matches.size()*params.correnspPerLineMatch);
  getCorrespondingPoints(source_coresp_points, target_coresp_points);
  correnspondences_time += watch.elapsed();

  watch.restart();
  Eigen::Matrix4f refinement = computeTransformationWeighted(source_coresp_points, target_coresp_points);
  target_cloud.transform(refinement);
  transformation = refinement * transformation;
  tranformation_time += watch.elapsed();

  watch.restart();
  float error = computeError(source_coresp_points, target_coresp_points, refinement);
  error_time += watch.elapsed();

  return error;
}

float CollarLinesRegistration::computeError() {
  findClosestMatchesByMiddles();

  MatrixOfPoints source_coresp_points(size_t(TPoint3D::RowsAtCompileTime),
                                      matches.size()*params.correnspPerLineMatch);
  MatrixOfPoints target_coresp_points(size_t(TPoint3D::RowsAtCompileTime),
                                      matches.size()*params.correnspPerLineMatch);
  cerr << "before" << endl;
  getCorrespondingPoints(source_coresp_points, target_coresp_points);
  cerr << "after" << endl;

  float error = computeError(source_coresp_points, target_coresp_points,
                             Eigen::Matrix4f::Identity());
  return error;
}

float CollarLinesRegistration::computeError(
    const MatrixOfPoints &source_coresp_points,
    const MatrixOfPoints &target_coresp_points,
    const Matrix4f &transformation) {
  typedef Eigen::Matrix<TPoint3D::Scalar, TPoint3D::RowsAtCompileTime+1, Eigen::Dynamic> MatrixOfHomogeniousPoints;
  MatrixOfHomogeniousPoints target_points_transformed =
      MatrixOfHomogeniousPoints::Ones(TPoint3D::RowsAtCompileTime+1, matches.size()*params.correnspPerLineMatch);
  target_points_transformed.block(0, 0, 3, matches.size()*params.correnspPerLineMatch) = target_coresp_points;
  target_points_transformed = transformation * target_points_transformed;
  MatrixOfPoints difference = source_coresp_points - target_points_transformed.block(0, 0, 3, matches.size()*params.correnspPerLineMatch);
  VectorXf square_distances = difference.cwiseProduct(difference).transpose() * Vector3f::Ones();
  return (square_distances.cwiseSqrt()).sum() / matches.size()*params.correnspPerLineMatch;
}

void CollarLinesRegistration::findClosestMatchesByMiddles() {
  matches.clear();

  for(int target_index = 0; target_index < target_cloud.line_cloud.size(); target_index++) {
    PointXYZ target_line_middle = target_cloud.line_middles[target_index];

    static const int K = 1;
    vector<int> closest_index(K);
    vector<float> min_distance(K);
    int matches_count = source_kdtree.
        nearestKSearch(target_line_middle, K, closest_index, min_distance);
    assert(matches_count == 1);

    // distance is actually square of real distance
    DMatch match(target_index, closest_index.front(), min_distance.front());
    matches.push_back(match);
  }

  float effective_threshold;
    if(params.distance_threshold == MEAN_THRESHOLD) {
      effective_threshold = getMatchesMean();
    } else if(params.distance_threshold == MEDIAN_THRESHOLD) {
      effective_threshold = getMatchesMedian();
    } else {
      assert(params.distance_threshold == NO_THRESHOLD);
      effective_threshold = INFINITY;
    }
  filterMatchesByThreshold(effective_threshold);
}

void CollarLinesRegistration::filterMatchesByThreshold(const float threshold) {
  vector<DMatch> old_matches = matches;
  matches.clear();
  for(vector<DMatch>::iterator m = old_matches.begin(); m < old_matches.end(); m++) {
    if(m->distance < threshold) {
      matches.push_back(*m);
    } else {
      rejected_matches.push_back(*m);
    }
  }
}

float CollarLinesRegistration::getMatchesMedian() {
  accumulators::accumulator_set<float, accumulators::stats<accumulators::tag::median > > acc;
  for(vector<DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
    acc(m->distance);
  }
  return accumulators::median(acc);
}

float CollarLinesRegistration::getMatchesMean() {
  float sum = 0;
  for(vector<DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
    sum += m->distance;
  }
  return sum / matches.size();
}

void CollarLinesRegistration::getCorrespondingPoints(
    MatrixOfPoints &source_coresp_points,
    MatrixOfPoints &target_coresp_points) {
  correspondences_weights = VectorXf(matches.size()*params.correnspPerLineMatch);
  int index = 0;
  for(vector<DMatch>::iterator match = matches.begin(); match < matches.end(); match++) {
    PointCloudLine source_line = source_cloud.line_cloud[match->trainIdx];
    PointCloudLine target_line = target_cloud.line_cloud[match->queryIdx];

    Vector3f source_line_pt, target_line_pt;
    source_line.closestPointsWith(target_line, source_line_pt, target_line_pt);
    RNG &rng = theRNG();
    for(int i = 0; i < params.correnspPerLineMatch; i++, index++) {

      Vector3f source_line_pt_noisy =
          source_line_pt + source_line.getOrientationOfSize(rng.gaussian(params.lineCorrenspSigma));
      Vector3f target_line_pt_noisy =
          target_line_pt + target_line.getOrientationOfSize(rng.gaussian(params.lineCorrenspSigma));

      if(false) {
        cerr << "source pt: " << source_line_pt_noisy.matrix() << endl;
        cerr << "target pt: " << target_line_pt_noisy.matrix() << endl;
        cerr << "--" << endl;
      }

      if(!(EigenUtils::allFinite(target_line_pt_noisy) && EigenUtils::allFinite(source_line_pt_noisy))) {
        source_line_pt_noisy = Vector3f(0,0,0);
        target_line_pt_noisy = Vector3f(0,0,0);
      }

      source_coresp_points.block(0, index, TPoint3D::RowsAtCompileTime, 1) = source_line_pt_noisy;
      target_coresp_points.block(0, index, TPoint3D::RowsAtCompileTime, 1) = target_line_pt_noisy;

      float weight;
      if(params.weighting == VERTICAL_ANGLE_WEIGHTS) {
        weight = getVerticalWeight(source_line.orientation, target_line.orientation);
      } else if(params.weighting == DISTANCE_WEIGHTS) {
        weight = 1/match->distance;
      } else {
        assert(params.weighting == NO_WEIGHTS);
        weight = 1;
      }
      correspondences_weights.data()[index] = weight;
    }
  }

  /* visualisation:
  Visualizer3D vis;
  for(int i = 0; i < matches.size(); i++) {
    if(i%3 == 0) {
      float w = MIN(1.0, correspondences_weights.data()[i]);
      PointCloudLine source_line = source_cloud.line_cloud[matches[i].trainIdx];
      vis.addLine(source_line, w, 0, 0);
    }
  }
  cerr << endl << endl;
  vis.show();*/
}

float CollarLinesRegistration::getVerticalWeight(const Vector3f &source_line_orient,
                                                   const Vector3f &target_line_orient) {
  static const float min_weight = 0.01;
  return (sinOfAngleWithGround(source_line_orient) *
      sinOfAngleWithGround(target_line_orient)) + min_weight;
}

float CollarLinesRegistration::sinOfAngleWithGround(const Vector3f &orientation) {
  return orientation.y() / orientation.norm();
}

void CollarLinesRegistration::getWeightingMatrix(WeightsMatrix &weighting_matrix) {
  if(correspondences_weights.size() == 0) {
    weighting_matrix.setIdentity();
    weighting_matrix.diagonal() /= (float)matches.size()*params.correnspPerLineMatch;
    return;
  }

  correspondences_weights /= correspondences_weights.sum();

  assert(weighting_matrix.rows() == correspondences_weights.size());
  assert(weighting_matrix.cols() == correspondences_weights.size());

  weighting_matrix.diagonal() = correspondences_weights;
}


Eigen::Matrix4f CollarLinesRegistration::computeTransformationWeighted(
    const MatrixOfPoints &source_coresp_points,
    const MatrixOfPoints &target_coresp_points)
{
  WeightsMatrix weights(correspondences_weights.size());
  getWeightingMatrix(weights);

  // Lets compute the translation
  // Define Column vector using definition of TPoint3D
  TPoint3D centroid_0;
  MatrixOfPoints target_points_weighted = target_coresp_points * weights;
  centroid_0 << target_points_weighted.row(0).sum(),
      target_points_weighted.row(1).sum(), target_points_weighted.row(2).sum();

  TPoint3D centroid_1;
  MatrixOfPoints source_points_weighted = source_coresp_points * weights;
  centroid_1 << source_points_weighted.row(0).sum(),
      source_points_weighted.row(1).sum(), source_points_weighted.row(2).sum();

  Eigen::Matrix<TPoint3D::Scalar, 1, Eigen::Dynamic, Eigen::Aligned> identity_vec = Eigen::Matrix<TPoint3D::Scalar, 1,
      Eigen::Dynamic>::Ones(1, target_coresp_points.cols()); //setOnes();

  // Create matrix with repeating values in columns
  MatrixOfPoints translate_0_mat = centroid_0 * identity_vec;
  MatrixOfPoints translate_1_mat = centroid_1 * identity_vec;

  // Translation of source_coresp_points to the target_coresp_points (Remember this is opposite of camera movement)
  // ie if camera is moving forward, the translation of target_coresp_points to source_coresp_points is opposite
  // TPoint3D t = (centroid_1 - centroid_0);

  // Translate the point cloud 0 to the coordinates of point cloud 1
  MatrixOfPoints target_coresp_points_translated(target_coresp_points.rows(), target_coresp_points.cols());
  MatrixOfPoints source_coresp_points_translated(source_coresp_points.rows(), source_coresp_points.cols());

  target_coresp_points_translated = target_coresp_points - translate_0_mat;
  source_coresp_points_translated = source_coresp_points - translate_1_mat;

  // Compute the Covariance matrix of these two pointclouds moved to the origin
  // This is not properly covariance matrix as there is missing the 1/N
  // 1/N is important for computing eigenvalues(scale), not the eigenvectors(directions) - as we are interested in eigenvectors

  Matrix3f A = target_coresp_points_translated * weights * source_coresp_points_translated.transpose();

  // Compute the SVD upon A = USV^t
  Eigen::JacobiSVD<Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Compute the determinant of V*U^t - to find out in what direction the rotation is
  float det = (svd.matrixV() * svd.matrixU().transpose()).determinant();

  // Fix the right hand/left hand rotation : assuming we would like the right hand rotation
  Matrix3f E = Matrix3f::Identity();
  E(2, 2) = (det >= 0) ? 1.0f : -1.0f;

  // Compute the rotation as R = VEU^t
  // R is the rotation of point_0_translated to fit the source_coresp_points_translated
  Matrix3f R = svd.matrixV() * E * (svd.matrixU().transpose());

  typedef Eigen::Matrix<TPoint3D::Scalar, 4, 1> _TyVector4;
  Eigen::Matrix4f transformation = _TyVector4::Ones().asDiagonal();
  transformation.block(0, 0, 3, 3) = R;

  // The translation must be computed as centroid_1 - rotated centroid_0
  transformation.block(0, 3, 3, 1) = centroid_1 - (R * centroid_0);

  return transformation;
}

void CollarLinesRegistration::showLinesCorrenspondences() {
  vector<DMatch> matches_in_plane;
  float min_distance = INFINITY;
  float max_distance = -1;
  for(vector<DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
    PointCloudLine source_line = source_cloud.line_cloud[m->trainIdx];
    PointCloudLine target_line = target_cloud.line_cloud[m->queryIdx];
    float distance = source_line.distanceTo(target_line, PointCloudLine::OF_CLOSEST_POINTS);
    matches_in_plane.push_back(DMatch(m->queryIdx, m->trainIdx, distance));
    min_distance = MIN(min_distance, distance);
    max_distance = MAX(max_distance, distance);
  }

  Visualizer3D visualizer;
  for(vector<DMatch>::iterator m = matches_in_plane.begin(); m < matches_in_plane.end(); m++) {
    PointCloudLine source_line = source_cloud.line_cloud[m->trainIdx];
    PointCloudLine target_line = target_cloud.line_cloud[m->queryIdx];
    float distance = (m->distance-min_distance)/(max_distance-min_distance);    // [0;1]
    visualizer.addLine(source_line, distance, 0.0, 0.0);
    visualizer.addLine(target_line, 0.0, distance, 0.0);
  }

  for(vector<DMatch>::iterator m = rejected_matches.begin(); m < rejected_matches.end(); m++) {
    PointCloudLine source_line = source_cloud.line_cloud[m->trainIdx];
    PointCloudLine target_line = target_cloud.line_cloud[m->queryIdx];
    visualizer.addLine(source_line, 1.0, 1.0, 1.0);
    visualizer.addLine(target_line, 1.0, 1.0, 1.0);
  }
  visualizer.show();
}


} /* namespace but_velodyne */
