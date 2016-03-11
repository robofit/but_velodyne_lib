/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 14/01/2015
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

#include <but_velodyne/PointCloudLine.h>

namespace but_velodyne {

using namespace Eigen;

Vector3f PointCloudLine::distanceVectorFrom(const PointCloudLine &other) const {
  Vector3f w0;
  float sc;
  float tc;
  closestPointsCoefficients(other, w0, sc, tc);
  return w0 + sc*this->orientation - tc*other.orientation;
}

bool PointCloudLine::exists() {
  return point[0] != 0.0 || point[1] != 0.0 || point[2] != 0.0 ||
      orientation[0] != 0.0 || orientation[1] != 0.0 || orientation[2] != 0.0;
}

ImageLine PointCloudLine::project(const cv::Mat &projection, const cv::Rect &frame) {
  cv::Point2f img_p1, img_p2;
  velodyne_pointcloud::PointXYZIR pcl_p1, pcl_p2;

  eigen2pcl(point, pcl_p1);
  eigen2pcl(point+orientation, pcl_p2);

  projectPoint(pcl_p1, projection, frame, img_p1);
  projectPoint(pcl_p2, projection, frame, img_p2);

  return ImageLine(img_p1, img_p2);
}

velodyne_pointcloud::PointXYZIR PointCloudLine::getBeginPoint() const {
  velodyne_pointcloud::PointXYZIR begin_point;
  eigen2pcl(point, begin_point);
  return begin_point;
}

velodyne_pointcloud::PointXYZIR PointCloudLine::getEndPoint() const {
  velodyne_pointcloud::PointXYZIR end_point;
  eigen2pcl(point+orientation, end_point);
  return end_point;
}

void PointCloudLine::inverse() {
  for(int i = 0; i < Eigen::Vector3f::RowsAtCompileTime; i++) {
    orientation[i] *= -1.0;
  }
}

PointCloudLine PointCloudLine::rotate(const Eigen::Matrix3f &R) const {
  PointCloudLine rotated;
  rotated.orientation = R*orientation;
  rotated.point = R*point;
  return rotated;
}

PointCloudLine PointCloudLine::transform(const Eigen::Matrix4f &transformation) const {
  Eigen::Vector3f t = transformation.block(0,3,3,1);
  Eigen::Matrix3f R = transformation.block(0,0,3,3);
  return transform(R, t);
}

PointCloudLine PointCloudLine::transform(const Eigen::Matrix3f &R, const Eigen::Vector3f &t) const {
  PointCloudLine transformed = this->rotate(R);
  transformed.point += t;
  return transformed;
}

void PointCloudLine::closestPointsWith(const PointCloudLine &other,
                       Eigen::Vector3f &this_pt, Eigen::Vector3f &other_pt) const {
  Eigen::Vector3f w0;
  float sc;
  float tc;
  closestPointsCoefficients(other, w0, sc, tc);
  this_pt = this->point + this->orientation*sc;
  other_pt = other.point + other.orientation*tc;
}

Eigen::Vector3f PointCloudLine::middle() const {
  return point + orientation*0.5;
}

float PointCloudLine::distanceTo(const PointCloudLine &other,
                                 DISTANCE distance_type) const {
  float similarity;
  switch(distance_type) {
    case COSINE_ORIENTATION:
      similarity = orientation.dot(other.orientation) /
        (orientation.norm()*other.orientation.norm());
      return (1.0 - similarity) / 2.0;
    case OF_MIDDLE_POINTS:
      return (this->middle() - other.middle()).norm();
    case EUCLIDEAN:
    case OF_CLOSEST_POINTS:
      return this->distanceVectorFrom(other).norm();
    default:
      assert(false);
      return 0;
  }
}

void PointCloudLine::closestPointsCoefficients(const PointCloudLine &other,
                               Eigen::Vector3f &w0, float &sc, float &tc) const {
  w0 = this->point - other.point;

  float a = this->orientation.dot(this->orientation);
  float b = this->orientation.dot(other.orientation);
  float c = other.orientation.dot(other.orientation);
  float d = this->orientation.dot(w0);
  float e = other.orientation.dot(w0);

  float denominator = a*c-b*b;
  sc = (b*e-c*d)/denominator;
  tc = (a*e-b*d)/denominator;
}

}
