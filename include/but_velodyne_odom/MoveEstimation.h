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

#ifndef MOVEESTIMATION_H_
#define MOVEESTIMATION_H_

#include <cv.h>

#include <pcl/common/eigen.h>
#include <boost/circular_buffer.hpp>
#include <boost/range/numeric.hpp>

namespace but_velodyne_odom
{

class MoveParameters
{
public:
  MoveParameters(float x, float y, float z, float roll, float pitch, float yaw) :
    x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {
  }

  MoveParameters(const Eigen::Matrix4f &t) {
    x = t.data()[12];
    y = t.data()[13];
    z = t.data()[14];

    Eigen::Affine3f affine(t);
    pcl::getEulerAngles(affine, roll, pitch, yaw);
  }

  MoveParameters operator +(const MoveParameters &other) const {
    return MoveParameters(
      x + other.x,
      y + other.y,
      z + other.z,
      roll + other.roll,
      pitch + other.pitch,
      yaw + other.yaw);
  }

  MoveParameters operator *(float factor) const {
    return MoveParameters(
        x * factor,
        y * factor,
        z * factor,
        roll * factor,
        pitch * factor,
        yaw * factor);
  }

  MoveParameters operator -(const MoveParameters &other) const {
    return *this + (other * -1.0f);
  }

  void operator +=(const MoveParameters &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    roll += other.roll;
    pitch += other.pitch;
    yaw += other.yaw;
  }

  void operator /=(float factor) {
    x /= factor;
    y /= factor;
    z /= factor;
    roll /= factor;
    pitch /= factor;
    yaw /= factor;
  }

  void operator *=(float factor) {
    x *= factor;
    y *= factor;
    z *= factor;
    roll *= factor;
    pitch *= factor;
    yaw *= factor;
  }

  void setZeros() {
    *this *= 0.0;
  }

  cv::Mat toCvMat() const {
    cv::Mat measurements(6, 1, CV_64F);
    measurements.at<double>(0) = x;
    measurements.at<double>(1) = y;
    measurements.at<double>(2) = z;
    measurements.at<double>(3) = roll;
    measurements.at<double>(4) = pitch;
    measurements.at<double>(5) = yaw;
    return measurements;
  }

  static void getGaussDistributionOf(const std::vector<MoveParameters> &meassurements,
                                     MoveParameters &output_mean,
                                     cv::Mat &output_covariance) {
    output_mean.setZeros();
    for(std::vector<MoveParameters>::const_iterator m = meassurements.begin();
          m < meassurements.end(); m++) {
      output_mean += *m;
    }
    output_mean /= meassurements.size();

    output_covariance = cv::Mat::zeros(6, 6, CV_64F);
    for(std::vector<MoveParameters>::const_iterator m = meassurements.begin();
              m < meassurements.end(); m++) {
      cv::Mat diff = (*m - output_mean).toCvMat();
      output_covariance += diff * diff.t();
    }
    output_covariance /= meassurements.size();
    std::cerr << "Covariance:" << std::endl;
    std::cerr << output_covariance << std::endl;
  }

  static void getDummyCovariance(const float t_diff,    // [m]
                                 const float r_diff,    // [rad]
                                 cv::Mat &output_covariance) {
    MoveParameters params(t_diff, t_diff, t_diff,
                          r_diff, r_diff, r_diff);
    output_covariance = params.toCvMat() * params.toCvMat().t();
  }

  float x, y, z, roll, yaw, pitch;
};

class MoveEstimator {
public:
  virtual ~MoveEstimator() {
  }
  virtual void addMeassurement(const MoveParameters &params) =0;
  virtual MoveParameters predict() =0;
};

class LinearMoveEstimator : public MoveEstimator {
public:
  LinearMoveEstimator(int history_size);
  virtual void addMeassurement(const MoveParameters &params);
  virtual MoveParameters predict();
private:
  boost::circular_buffer<MoveParameters> last_measurements;
};

class KalmanMoveEstimator : public MoveEstimator {
public:
  KalmanMoveEstimator(const float process_noise, const float measurement_noise,
                      const float error_variance);
  virtual void addMeassurement(const MoveParameters &params);
  virtual MoveParameters predict();
private:
  cv::KalmanFilter KF;
  MoveParameters estimation;

  static const int STATES = 18;
  static const int MEASSURES_STATES = 6;
  static const int ACTION_CONTROLS = 0;
  static const float TIME_DELTA = 0.1;         // 10fps
};

class MoveEstimation
{
public:
  MoveEstimation(MoveEstimator &estimator);
  void addMeassurement(Eigen::Matrix4f &m);
  Eigen::Matrix4f predict();
private:
  MoveEstimator &estimator;
};

} /* namespace but_velodyne_odom */

#endif /* MOVEESTIMATION_H_ */
