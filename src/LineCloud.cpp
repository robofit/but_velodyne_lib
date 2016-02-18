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

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/program_options/errors.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <but_velodyne/LineCloud.h>

using namespace std;
using namespace pcl;

namespace but_velodyne
{

std::istream& operator>> (std::istream &in, LineCloud::PreservedFactorBy &factor_type) {
  string token;
  in >> token;

  boost::to_upper(token);

  if (token == "ANGLE_WITH_GROUND") {
    factor_type = LineCloud::ANGLE_WITH_GROUND;
  } else if (token == "NONE") {
    factor_type = LineCloud::NONE;
  } else {
      throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value,
                                                     "lines_preserved_factor_by");
  }

  return in;
}


LineCloud::LineCloud(const PolarGridOfClouds &polar_grid,
                     const int lines_per_cell_pair_generated,
                     const int lines_per_cell_pair_preserved,
                     const PreservedFactorBy preservedFactorType) :
    rng(cv::theRNG()),
    lines_per_cell_pair_generated(lines_per_cell_pair_generated),
    lines_per_cell_pair_preserved(lines_per_cell_pair_preserved),
    preservedFactorType(preservedFactorType)
{
  for(int polar = 0; polar < PolarGridOfClouds::POLAR_BINS; polar++) {
    for(int ring = 0; ring < VelodynePointCloud::VELODYNE_RINGS_COUNT-1; ring++) {
      vector<PointCloudLine> lines_among_cells;
      generateLineCloudAmongCells(polar_grid,
                                  CellId(polar, ring), CellId(polar, ring+1),
                                  lines_among_cells);
      line_cloud.insert(line_cloud.end(),
                        lines_among_cells.begin(), lines_among_cells.end());
      for(vector<PointCloudLine>::iterator line = lines_among_cells.begin();
          line < lines_among_cells.end();
          line++) {
        Eigen::Vector3f middle = line->middle();
        line_middles.push_back(PointXYZ(middle.x(), middle.y(), middle.z()));
      }
    }
  }
}

void LineCloud::generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                            const CellId &cell1_id, const CellId &cell2_id,
                                            vector<PointCloudLine> &output_lines) {
  output_lines.clear();

  const VelodynePointCloud& cell1 = polar_grid[cell1_id];
  const VelodynePointCloud& cell2 = polar_grid[cell2_id];
  int lines_to_generate = MIN(lines_per_cell_pair_generated, cell1.size()*cell2.size());
  VelodynePointCloud all_points;
  all_points += cell1; all_points += cell2;
  float preserved_factor = getPreservedFactor(all_points);
  int lines_to_preserve = MIN(lines_per_cell_pair_preserved*preserved_factor,
                              lines_to_generate);
  assert(lines_to_generate >= lines_to_preserve);

  for(int i = 0; i < lines_to_generate; i++) {
    int cell1_index = rng(cell1.size());
    int cell2_index = rng(cell2.size());
    PointCloudLine generated_line(cell1[cell1_index],
                                  cell2[cell2_index]);
    output_lines.push_back(generated_line);
  }
  sort(output_lines.begin(), output_lines.end());
  output_lines.erase(output_lines.begin()+lines_to_preserve, output_lines.end());
}

float LineCloud::sinOfPlaneAngleWithGround(const VelodynePointCloud &points) {
  NormalEstimation<PointXYZ, Normal> normal_est;
  vector<int> indices;
  for(int i = 0; i < points.size(); i++) {
    indices.push_back(i);
  }
  float nx, ny, nz, c;
  normal_est.computePointNormal(*(points.getXYZCloudPtr()), indices, nx, ny, nz, c);
  float angle_cos = ny / sqrt(nx*nx + ny*ny + nz*nz);
  float angle_sin = sqrt(1 - angle_cos*angle_cos);
  return (isnan(angle_sin)) ? 1.0 : angle_sin;
}

void LineCloud::transform(const Eigen::Matrix4f &transformation, LineCloud &output) const {
  output.line_cloud.clear();
  for(std::vector<PointCloudLine>::const_iterator line = line_cloud.begin();
      line < line_cloud.end(); line++) {
    output.line_cloud.push_back(line->transform(transformation));
  }
  pcl::transformPointCloud(line_middles, output.line_middles, transformation);
}

void LineCloud::transform(const Eigen::Matrix4f &transformation) {
  for(std::vector<PointCloudLine>::iterator line = line_cloud.begin();
      line < line_cloud.end(); line++) {
    *line = line->transform(transformation);
  }
  pcl::transformPointCloud(line_middles, line_middles, transformation);
}

float LineCloud::getPreservedFactor(const VelodynePointCloud &all_points) {
  switch(preservedFactorType) {
    case ANGLE_WITH_GROUND:
      return sinOfPlaneAngleWithGround(all_points) + 1.0;
    default:
      assert(preservedFactorType == NONE);
      return 1.0;
  }
}

} /* namespace but_velodyne */
