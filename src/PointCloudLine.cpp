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

#include <but_velodyne_odom/PointCloudLine.h>

namespace but_velodyne_odom {

using namespace Eigen;

Vector3f PointCloudLine::distanceVectorFrom(const PointCloudLine &other) const {
  Vector3f w0;
  float sc;
  float tc;
  closestPointsCoefficients(other, w0, sc, tc);
  return w0 + sc*this->orientation - tc*other.orientation;
}

Eigen::Vector3f PointCloudLine::middle() const {
  return point + orientation*0.5;
}

float PointCloudLine::distanceTo(const PointCloudLine &other,
                                 DISTANCE distance_type) const {
  float similarity;
  switch(distance_type) {
    case EUCLIDIAN:
      return this->distanceVectorFrom(other).norm();
    case COSINE_ORIENTATION:
      similarity = orientation.dot(other.orientation) /
        (orientation.norm()*other.orientation.norm());
      return (1.0 - similarity) / 2.0;
    case OF_MIDDLE_POINTS:
      return (this->middle() - other.middle()).norm();
    case OF_CLOSEST_POINTS:
      return this->distanceVectorFrom(other).norm();
    default:
      assert(false);
      return 0;
  }
}

}
