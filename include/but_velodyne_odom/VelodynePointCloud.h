/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2014
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

#ifndef VELODYNEPOINTCLOUD_H_
#define VELODYNEPOINTCLOUD_H_

#include <fstream>

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>

#include <cv.h>

namespace but_velodyne_odom {

inline float computeRange(const velodyne_pointcloud::PointXYZIR &pt)
{
  return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

template<typename PclPointT>
inline Eigen::Vector3f pclPointToVector3f(const PclPointT &pt) {
  return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

inline velodyne_pointcloud::PointXYZIR PointXYZ2PointXYZIR(const pcl::PointXYZ &pt)
{
  velodyne_pointcloud::PointXYZIR pt_ir;
  pt_ir.x = pt.x;
  pt_ir.y = pt.y;
  pt_ir.z = pt.z;
  pt_ir.intensity = 0;
  pt_ir.ring = 0;
  return pt_ir;
}

pcl::PointXYZ PointXYZIRtoPointXYZ(const velodyne_pointcloud::PointXYZIR &in);
bool projectPoint(const velodyne_pointcloud::PointXYZIR &pt,
                  const cv::Mat &projection_matrix,
                  const cv::Rect &plane,
                  cv::Point2f &projected_pt);

class VelodynePointCloud : public pcl::PointCloud<velodyne_pointcloud::PointXYZIR>
{
public:
  void normalizeIntensity(float min_intensity, float max_intensity);
  VelodynePointCloud computeEdges(float threshold) const;
  velodyne_pointcloud::PointXYZIR getMinValuePt() const;
  velodyne_pointcloud::PointXYZIR getMaxValuePt() const;
  VelodynePointCloud resampleTo(int final_number);
  VelodynePointCloud resampleByRatio(float preserve_ratio);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZCloudPtr() const;

  float getMedianRange() const;

  void setImageLikeAxisFromKitti();
  void setImageLikeAxisFromBut();

  static void fromKitti(const std::string &infile,
                        VelodynePointCloud &out_cloud) {
          out_cloud.clear();
          // load point cloud
          std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
          if(!input.good()){
                  std::cerr << "Could not read file: " << infile << std::endl;
                  exit(EXIT_FAILURE);
          }
          input.seekg(0, std::ios::beg);

          int i;
          for (i=0; input.good() && !input.eof(); i++) {
                  velodyne_pointcloud::PointXYZIR point;
                  input.read((char *) &point.x, 3*sizeof(float));
                  input.read((char *) &point.intensity, sizeof(float));
                  out_cloud.push_back(point);
          }
          input.close();

          int ring = 0;
          int ring_size = out_cloud.size() / VELODYNE_RINGS_COUNT;
          for(int i = 0; i < out_cloud.size(); i++) {
            if(i != 0 && (i % ring_size) == 0) {
              ring++;
            }

            if(ring < VELODYNE_RINGS_COUNT) {
              out_cloud[i].ring = ring;
            } else {
              out_cloud.erase(out_cloud.begin()+i, out_cloud.end());
            }
          }
          out_cloud.setImageLikeAxisFromKitti();

          std::cerr << "Read KTTI point cloud " << infile
              << " with " << i << " points." << std::endl;
  }

protected:
  std::vector< std::vector<velodyne_pointcloud::PointXYZIR> > getRings() const;
  VelodynePointCloud discartWeakPoints(float threshold);

public:
  static const uint16_t VELODYNE_RINGS_COUNT = 64;
};

}

#endif /* VELODYNEPOINTCLOUD_H_ */
