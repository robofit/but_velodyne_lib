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

#ifndef POLARGRIDOFCLOUDS_H_
#define POLARGRIDOFCLOUDS_H_

#include <velodyne_pointcloud/point_types.h>
#include <boost/array.hpp>
#include <cv.h>
#include <pcl/common/eigen.h>

#include <but_velodyne/VelodynePointCloud.h>

namespace but_velodyne
{

/**!
 * Identifier of the polar cell.
 */
struct CellId {
  int ring;     ///! Velodyne ring ID
  int polar;    ///! discretized horizontal angle

  CellId(int p, int r) :
    ring(r), polar(p) {
  }
};

/**!
 * Polar grid structure grouping the Velodyne 3D points of the same ring and the similar horizontal angle.
 */
class PolarGridOfClouds
{
public:

  typedef boost::shared_ptr<PolarGridOfClouds> Ptr;

  /**!
   * Redistributes the original 3D points into the polar grid structure
   *
   * @param point_cloud oroginal Velodyne point cloud
   */
  PolarGridOfClouds(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud);

  static Ptr of(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud) {
    PolarGridOfClouds *grid = new PolarGridOfClouds(point_cloud);
    return Ptr(grid);
  }

  /**!
   * Visualization of the grouping into the polar bins - same bin = same color.
   */
  void showColored();

  /**!
   * @param cellId identifier of the cell
   * @returns the points of the specific polar grid
   */
  const VelodynePointCloud& operator[](const CellId &cellId) const {
    return polar_grid[cellId.polar][cellId.ring];
  }

  /**!
   * Transforms the positions of points in the grid structure. Distribution of points
   * into the polar bins remains the same since only rigid transformation is assumed.
   *
   * @param t rigid 3D transformation [R|t]
   */
  void transform(const Eigen::Matrix4f &t);

  static const int POLAR_BINS = 36;     //!! number of polar bins (10deg each)

protected:

  void fill(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud);

  float getPolarAngle(float x, float y)
  {
    static const float rad_to_deg = 180.0f / float(CV_PI);
    return std::atan2(y, x) * rad_to_deg;
  }

  int getPolarBinIndex(const velodyne_pointcloud::PointXYZIR &point);

  boost::array<
    boost::array<VelodynePointCloud, VelodynePointCloud::VELODYNE_RINGS_COUNT>,
    POLAR_BINS > polar_grid;            // polar bins of rings
};

} /* namespace but_velodyne */

#endif /* POLARGRIDOFCLOUDS_H_ */
