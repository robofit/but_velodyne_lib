/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 22/06/2015
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

#ifndef POSELOOP_H_
#define POSELOOP_H_

#include <cv.h>
#include <pcl/common/eigen.h>

#include <but_velodyne/MoveEstimation.h>

namespace but_velodyne
{

/**!
 * The edge (representing transformation) in the pose graph
 * connecting two vertices (two positions).
 */
class PoseGraphEdge
{
public:

  /**!
   * @param sourceIdx ID of source vertex the edge is connected to
   * @param targetIdx ID of target vertex the edge is connected to
   * @param transformation transformation between the positions (vertices)
   * @param covariance covariance matrix for SLAM++ (how much the algorithm trusts the edge)
   */
  PoseGraphEdge(int sourceIdx, int targetIdx,
                const Eigen::Matrix4f transformation, const cv::Mat &covariance) :
    sourceIdx(sourceIdx), targetIdx(targetIdx),
    transformation(transformation), covariance(covariance) {
  }

  /**!
   * @param sourceIdx ID of source vertex the edge is connected to
   * @param targetIdx ID of target vertex the edge is connected to
   * @param transformation transformation between the positions (vertices)
   */
  PoseGraphEdge(int sourceIdx, int targetIdx,
                const Eigen::Matrix4f transformation) :
    sourceIdx(sourceIdx), targetIdx(targetIdx),
    transformation(transformation) {
    MoveParameters::getDummyCovariance(0.01, 0.02, covariance);
  }

  /**!
   * Ordering of the edges by vertices indices.
   */
  bool operator<(const PoseGraphEdge &other) const {
    if(this->sourceIdx == other.sourceIdx) {
      return this->targetIdx < other.targetIdx;
    }
    return this->sourceIdx < other.sourceIdx;
  }

  int sourceIdx;                        ///! ID of source vertex the edge is connected to
  int targetIdx;                        ///! ID of target vertex the edge is connected to
  Eigen::Matrix4f transformation;       ///! transformation between the positions (vertices)
  cv::Mat covariance;                   ///! covariance matrix for SLAM++ (how much the algorithm trusts the edge)
};

/**!
 * Prints the edge in SLAM++ format:
 * EDGE3D sourceIdx targetIdx 6DOF(6 floats) upper-triange-of-cholesky-cov(21 floats)
 */
std::ostream& operator<<(std::ostream &os, const PoseGraphEdge &obj);

} /* namespace but_velodyne */

#endif /* POSELOOP_H_ */
