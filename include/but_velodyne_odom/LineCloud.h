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

#ifndef LINESCLOUD_H_
#define LINESCLOUD_H_

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne_odom/PointCloudLine.h>
#include <but_velodyne_odom/PolarGridOfClouds.h>

namespace but_velodyne_odom
{

class LineCloud
{
public:
  enum PreservedFactorBy {
    ANGLE_WITH_GROUND,
    NONE
  };

  LineCloud() :
    rng(cv::theRNG()),
    lines_per_cell_pair_generated(-1),
    lines_per_cell_pair_preserved(-1),
    preservedFactorType(NONE) {
    // empty
  }

  LineCloud(const PolarGridOfClouds &polar_grid,
            const int lines_per_cell_pair_generated,
            const int lines_per_cell_pair_preserved,
            const PreservedFactorBy preservedFactorType);

  const std::vector<PointCloudLine>& getLines() const {
    return line_cloud;
  }

  void transform(const Eigen::Matrix4f &transformation, LineCloud &output) const {
    output.line_cloud.clear();
    for(std::vector<PointCloudLine>::const_iterator line = line_cloud.begin();
        line < line_cloud.end(); line++) {
      output.line_cloud.push_back(line->transform(transformation));
    }
    pcl::transformPointCloud(line_middles, output.line_middles, transformation);
  }

  void transform(const Eigen::Matrix4f &transformation) {
    for(std::vector<PointCloudLine>::iterator line = line_cloud.begin();
        line < line_cloud.end(); line++) {
      *line = line->transform(transformation);
    }
    pcl::transformPointCloud(line_middles, line_middles, transformation);
  }

  std::vector<PointCloudLine> line_cloud;
  pcl::PointCloud<pcl::PointXYZ> line_middles;

protected:
  void generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                   const CellId &cell1, const CellId &cell2,
                                   std::vector<PointCloudLine> &line_cloud);

  float sinOfPlaneAngleWithGround(const VelodynePointCloud &points);

  float getPreservedFactor(const VelodynePointCloud &all_points) {
    switch(preservedFactorType) {
      case ANGLE_WITH_GROUND:
        return sinOfPlaneAngleWithGround(all_points) + 1.0;
      default:
        assert(preservedFactorType == NONE);
        return 1.0;
    }
  }

private:
  cv::RNG& rng;
  const int lines_per_cell_pair_generated;
  const int lines_per_cell_pair_preserved;
  const PreservedFactorBy preservedFactorType;
};

} /* namespace but_velodyne_odom */

#endif /* LINESCLOUD_H_ */
