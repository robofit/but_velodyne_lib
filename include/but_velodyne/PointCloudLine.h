/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 12/01/2015
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

#ifndef POINTCLOUDLINE_H_
#define POINTCLOUDLINE_H_

#include <but_velodyne/ImageLine.h>
#include <but_velodyne/VelodynePointCloud.h>

namespace but_velodyne {

/**!
 * 3D line (or line segment AB) representation.
 */
class PointCloudLine {
public:
  Eigen::Vector3f point;        ///! point on the line (or A endpoint of line segment)
  Eigen::Vector3f orientation;  ///! orientation vector (for line segment AB: orientation=B-A)

  /**!
   * Types of distance between two lines.
   */
  enum DISTANCE {
    COSINE_ORIENTATION, ///! cosine distance of orientations
    EUCLIDEAN,          ///! Euclidean distance of the closest points
    OF_MIDDLE_POINTS,   ///! Euclidean distance of middle points (point+orientation/2)
    OF_CLOSEST_POINTS   ///! same as EUCLIDEAN
  };

public:

  /**!
   * @param point endpoint
   * @pamra orientation orientation vector. In case of line segment is the length of this vector same as length of the segment.
   */
  PointCloudLine(Eigen::Vector3f point, Eigen::Vector3f orientation) :
    point(point), orientation(orientation) {
  }

  /**!
   * Creates line, where A = B = [0,0,0] (not valid one)
   */
  PointCloudLine() {
    point.setZero();
    orientation.setZero();
  }

  /**!
   * @param coefficients vector [p_x, p_y, p_z, o_x, o_y, o_z] where [p_x, p_y, p_z] is the endpoint and [o_x, o_y, o_z] the orientation
   */
  PointCloudLine(const Eigen::VectorXf &coefficients) {
    setCoefficients(coefficients);
  }

  /**!
   * Create line (or line segment) between two PCL 3D points.
   */
  template<typename PclPointT>
  PointCloudLine(const PclPointT &p1,
                 const PclPointT &p2) {
    point = pclPointToVector3f(p1);
    orientation = pclPointToVector3f(p2) - point;
  }

  /**!
   * Reset the position and the orientation of the line
   *
   * @param coefficients vector [p_x, p_y, p_z, o_x, o_y, o_z] where [p_x, p_y, p_z] is the endpoint and [o_x, o_y, o_z] the orientation
   */
  void setCoefficients(const Eigen::VectorXf &coefficients) {
    for(int i = 0; i < 6; i++) {
      if(i < 3) {
        point[i] = coefficients[i];
      } else {
        orientation[i-3] = coefficients[i];
      }
    }
  }

  /**!
   * Returns invalid line.
   */
  static PointCloudLine None() {
    PointCloudLine none;
    return none;
  }

  /**!
   * @return true iff the line is valid.
   */
  bool exists();

  /**!
   * Projects the 3D line segment into the image plane.
   *
   * @param projection 3x4 projection matrix
   * @param frame size of image
   * @return projected 2D line segment
   */
  ImageLine project(const cv::Mat &projection, const cv::Rect &frame);

  static void eigen2pcl(const Eigen::Vector3f &eigen,
                        velodyne_pointcloud::PointXYZIR &pcl) {
    pcl.x = eigen[0];
    pcl.y = eigen[1];
    pcl.z = eigen[2];
  }

  /**!
   * @return first endpoint of the line segment as a PCL 3D point of Velodyne LiDAR
   */
  velodyne_pointcloud::PointXYZIR getBeginPoint() const;

  /**!
   * @return second endpoint of the line segment as a PCL 3D point of Velodyne LiDAR
   */
  velodyne_pointcloud::PointXYZIR getEndPoint() const;

  /**!
   * Method switches the endpoints of line segment (AB -> BA)
   */
  void inverse();

  /**!
   * 3D rotation of the line segment.
   *
   * @param R rotation matrix
   * @return rotated line segment
   */
  PointCloudLine rotate(const Eigen::Matrix3f &R) const;

  /**!
   * 3D rigid transformation of the line segment.
   *
   * @param transformation transformation matrix
   * @return transformed line segment
   */
  PointCloudLine transform(const Eigen::Matrix4f &transformation) const;

  /**!
   * 3D rigid transformation of the line segment.
   *
   * @param R rotation matrix
   * @param t tranlation vector
   * @return transformed line segment
   */
  PointCloudLine transform(const Eigen::Matrix3f &R, const Eigen::Vector3f &t) const;

  /**!
   * Compares the lines by length
   */
  bool operator <(const PointCloudLine &other) const {
    return this->orientation.norm() < other.orientation.norm();
  }

  /**!
   * @param other second line segment
   * @return (P_t-P_o) where P_t and P_o are closest points of *this and the other line segment
   */
  Eigen::Vector3f distanceVectorFrom(const PointCloudLine &other) const;

  /**!
   * Method finds the closest points of two (*this and other) line segments
   *
   * @param other second line segment
   * @param this_pt [output] the closest point found on this line
   * @param other_pt [output] the closest point found on the other line
   */
  void closestPointsWith(const PointCloudLine &other,
                         Eigen::Vector3f &this_pt, Eigen::Vector3f &other_pt) const;

  /**!
   * @param other second line segment
   * @param distance_type type of distance @see DISTANCE
   * @return the distance between *this and the other line
   */
  float distanceTo(const PointCloudLine &other, DISTANCE distance_type) const;

  /**!
   * @return middle point of this line segment
   */
  Eigen::Vector3f middle() const;

  /**!
   * @param l desired vector length
   * @return the orientation vector of this line resized to the length l
   */
  Eigen::Vector3f getOrientationOfSize(float l) const {
    return orientation / orientation.norm() * l;
  }

protected:
  void closestPointsCoefficients(const PointCloudLine &other,
                                 Eigen::Vector3f &w0, float &sc, float &tc) const;
};

}

#endif /* POINTCLOUDLINE_H_ */
