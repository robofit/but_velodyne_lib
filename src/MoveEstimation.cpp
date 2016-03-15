/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 13/05/2015
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

#include <but_velodyne/MoveEstimation.h>

namespace but_velodyne
{

/* **************************** MoveParameters **************************** */

MoveParameters MoveParameters::operator +(const MoveParameters &other) const {
  return MoveParameters(
    x + other.x,
    y + other.y,
    z + other.z,
    roll + other.roll,
    pitch + other.pitch,
    yaw + other.yaw);
}

MoveParameters MoveParameters::operator *(float factor) const {
  return MoveParameters(
      x * factor,
      y * factor,
      z * factor,
      roll * factor,
      pitch * factor,
      yaw * factor);
}

MoveParameters MoveParameters::operator -(const MoveParameters &other) const {
  return *this + (other * -1.0f);
}

void MoveParameters::operator +=(const MoveParameters &other) {
  x += other.x;
  y += other.y;
  z += other.z;
  roll += other.roll;
  pitch += other.pitch;
  yaw += other.yaw;
}

void MoveParameters::operator /=(float factor) {
  x /= factor;
  y /= factor;
  z /= factor;
  roll /= factor;
  pitch /= factor;
  yaw /= factor;
}

void MoveParameters::operator *=(float factor) {
  x *= factor;
  y *= factor;
  z *= factor;
  roll *= factor;
  pitch *= factor;
  yaw *= factor;
}

void MoveParameters::setZeros() {
  *this *= 0.0;
}

cv::Mat MoveParameters::toCvMat() const {
  cv::Mat measurements(6, 1, CV_64F);
  measurements.at<double>(0) = x;
  measurements.at<double>(1) = y;
  measurements.at<double>(2) = z;
  measurements.at<double>(3) = roll;
  measurements.at<double>(4) = pitch;
  measurements.at<double>(5) = yaw;
  return measurements;
}

/* **************************** LinearMoveEstimator **************************** */

LinearMoveEstimator::LinearMoveEstimator(int history_size) :
  last_measurements(history_size) {
}

void LinearMoveEstimator::addMeassurement(const MoveParameters &parameters) {
  last_measurements.push_back(parameters);
}

MoveParameters LinearMoveEstimator::predict() {
  MoveParameters prediction(0, 0, 0, 0, 0, 0);
  if(last_measurements.size() != 0) {
    float factor = 0;
    float weight = 1;
    for(boost::circular_buffer<MoveParameters>::iterator m = last_measurements.begin();
            m < last_measurements.end(); m++, weight += 1.0) {
      prediction += (*m) * weight;
      factor += weight;
    }
    prediction /= factor;
  }
  return prediction;
}

/* ***************************** KalmanMoveEstimator **************************** */

KalmanMoveEstimator::KalmanMoveEstimator(const float process_noise,
                                         const float measurement_noise,
                                         const float error_variance) :
    estimation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {

  KF.init(STATES, MEASSURES_STATES, ACTION_CONTROLS, CV_64F);
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(process_noise));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(measurement_noise));
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(error_variance));

  // DYNAMIC MODEL:
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

  float dt = TIME_DELTA;
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);

  // MEASUREMENT MODEL
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}

void KalmanMoveEstimator::addMeassurement(const MoveParameters &params) {
  // First predict, to update the internal statePre variable
  cv::Mat prediction = KF.predict();

  // The "correct" phase that is going to use the predicted value and our measurement
  cv::Mat measurement = params.toCvMat();
  cv::Mat estimated = KF.correct(measurement);

  // Estimated translation
  estimation.x = estimated.at<double>(0);
  estimation.y = estimated.at<double>(1);
  estimation.z = estimated.at<double>(2);

  // Estimated euler angles
  estimation.roll = estimated.at<double>(9);
  estimation.pitch = estimated.at<double>(10);
  estimation.yaw = estimated.at<double>(11);
}

MoveParameters KalmanMoveEstimator::predict() {
  return estimation;
}
const int KalmanMoveEstimator::STATES = 18;
const int KalmanMoveEstimator::MEASSURES_STATES = 6;
const int KalmanMoveEstimator::ACTION_CONTROLS = 0;
const float KalmanMoveEstimator::TIME_DELTA = 0.1;         // 10fps

/* ******************************* MoveEstimation ******************************* */

MoveEstimation::MoveEstimation(MoveEstimator &estimator) :
    estimator(estimator) {
}

void MoveEstimation::addMeassurement(Eigen::Matrix4f &m) {
  MoveParameters parameters(m);
  estimator.addMeassurement(parameters);
}

Eigen::Matrix4f MoveEstimation::predict() {
  MoveParameters prediction = estimator.predict();
  return pcl::getTransformation(
      prediction.x,
      prediction.y,
      prediction.z,
      prediction.roll,
      prediction.pitch,
      prediction.yaw).matrix();
}

} /* namespace but_velodyne */
