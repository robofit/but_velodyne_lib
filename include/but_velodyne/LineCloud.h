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

#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/PointCloudLine.h>
#include <but_velodyne/PolarGridOfClouds.h>

namespace but_velodyne
{

/**!
 * Cloud of collar line segments.
 */
class LineCloud
{
public:

  /**!
   * Probability of line to be preserved may be based on the
   * angle of polar bin (estimated plane) with ground plane - vertical bins are preferred.
   */
  enum PreservedFactorBy {
    ANGLE_WITH_GROUND,
    NONE
  };

  friend std::istream& operator>> (std::istream &in, PreservedFactorBy &factor_type);

  /**!
   * Initialize empty line cloud.
   */
  LineCloud() :
    rng(cv::theRNG()),
    lines_per_cell_pair_generated(-1),
    lines_per_cell_pair_preserved(-1),
    preservedFactorType(NONE) {
    // empty
  }

  /**!
   * @param polar_grid point cloud formated into polar grid
   * @param lines_per_cell_pair_generated how many lines are generated for each bin
   * @param lines_per_cell_pair_preserved how many lines are preserved within each bin
   * @param preservedFactorType which line segments are preferred
   */
  LineCloud(const PolarGridOfClouds &polar_grid,
            const int lines_per_cell_pair_generated,
            const int lines_per_cell_pair_preserved,
            const PreservedFactorBy preservedFactorType);

  /**!
   * @return all lines
   */
  const std::vector<PointCloudLine>& getLines() const {
    return line_cloud;
  }

  /**!
   * Transform line cloud.
   *
   * @param transformation transformation matrix
   * @param output destination
   */
  void transform(const Eigen::Matrix4f &transformation, LineCloud &output) const;

  /**!
   * Transform line cloud - output is *this cloud.
   *
   * @param transformation transformation matrix
   */
  void transform(const Eigen::Matrix4f &transformation);

  std::vector<PointCloudLine> line_cloud;       ///! collar line segments
  pcl::PointCloud<pcl::PointXYZ> line_middles;  ///! midpoints of line segments

protected:
  void generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                   const CellId &cell1, const CellId &cell2,
                                   std::vector<PointCloudLine> &line_cloud);

  float sinOfPlaneAngleWithGround(const VelodynePointCloud &points);

  float getPreservedFactor(const VelodynePointCloud &all_points);

private:
  cv::RNG& rng;
  const int lines_per_cell_pair_generated;
  const int lines_per_cell_pair_preserved;
  const PreservedFactorBy preservedFactorType;
};

} /* namespace but_velodyne */

#endif /* LINESCLOUD_H_ */
