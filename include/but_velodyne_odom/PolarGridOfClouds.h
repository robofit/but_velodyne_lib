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

#include <but_velodyne_odom/VelodynePointCloud.h>

namespace but_velodyne_odom
{

struct CellId {
  int ring;
  int polar;

  CellId(int p, int r) :
    ring(r), polar(p) {
  }
};

class PolarGridOfClouds
{
public:

  typedef boost::shared_ptr<PolarGridOfClouds> Ptr;

  PolarGridOfClouds(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud);

  static Ptr of(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud) {
    PolarGridOfClouds *grid = new PolarGridOfClouds(point_cloud);
    return Ptr(grid);
  }

  void showColored();

  const VelodynePointCloud& operator[](const CellId &cellId) const {
    return polar_grid[cellId.polar][cellId.ring];
  }

  void transform(const Eigen::Matrix4f &t);

  static const int POLAR_BINS = 36;     // 10deg each

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

} /* namespace but_velodyne_odom */

#endif /* POLARGRIDOFCLOUDS_H_ */
