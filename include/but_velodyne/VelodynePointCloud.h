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

namespace but_velodyne {

/**!
 * @param pt Velodyne 3D point
 * @returns the distance of the point from sensor
 */
inline float computeRange(const velodyne_pointcloud::PointXYZIR &pt)
{
  return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

/**!
 * Conversion from PCL point to Eigen::Vector3f
 *
 * @param pt Velodyne 3D point
 * @return Eigen representation of 3D points
 */
template<typename PclPointT>
inline Eigen::Vector3f pclPointToVector3f(const PclPointT &pt) {
  return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

/**!
 * Conversion from PCL PointXYZ to PointXYZIR
 *
 * @param pt 3D point
 * @return PCL representation of Velodyne 3D point
 */
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
/**!
 * Conversion from PCL PointXYZIR to PointXYZ
 *
 * @param pt Velodyne 3D point
 * @return PCL 3D point
 */
pcl::PointXYZ PointXYZIRtoPointXYZ(const velodyne_pointcloud::PointXYZIR &in);

/**!
 * Projects the Velodyne 3D point to the image plane
 *
 * @param pt Velodyne 3D point
 * @param projection_matrix 3x4 projection matrix
 * @param plane image plane (size of the image is required)
 * @param projected_pt [output] projected 2D point
 * @return true iff the 3D point is projected within the image dimensions
 */
bool projectPoint(const velodyne_pointcloud::PointXYZIR &pt,
                  const cv::Mat &projection_matrix,
                  const cv::Rect &plane,
                  cv::Point2f &projected_pt);

/**!
 * Representation of the Velodyne LiDAR point cloud. The axis are arranged in following manner:
 *
 *    /^ Z-axis
 *   /
 *  /
 * +------> X-axis
 * |
 * |
 * |
 * V Y-axis
 */
class VelodynePointCloud : public pcl::PointCloud<velodyne_pointcloud::PointXYZIR>
{
public:
  /**!
   * Normalization of the point intensities to interval [min_intensity, max_intensity]
   *
   * @param min_intensity lower interval boundary
   * @param max_intensity upper interval boundary
   */
  void normalizeIntensity(float min_intensity, float max_intensity);

  /**!
   * Edge detection in the Velodyne point cloud. Detection searches for the
   * discontinuities along the ring of points.
   *
   * @return point cloud where the intensity of point is proportional to the discontinuity in this point
   */
  VelodynePointCloud computeEdges(float threshold) const;

  /**!
   * @return the point with minimal intensity value
   */
  velodyne_pointcloud::PointXYZIR getMinValuePt() const;

  /**!
   * @return the point with maximal intensity value
   */
  velodyne_pointcloud::PointXYZIR getMaxValuePt() const;

  /**!
   * Regularly resamples the point cloud.
   *
   * @param final_number expected number of preserved points
   * @return resampled point cloud with MIN(this->size(), final_number) points
   */
  VelodynePointCloud resampleTo(int final_number);

  /**!
   * Regularly resamples the point cloud.
   *
   * @param preserve_ratio ratio (espected_size/this->size())
   * @return resampled point cloud with MIN(this->size(), this->size()*preserve_ratio) points
   */
  VelodynePointCloud resampleByRatio(float preserve_ratio);

  /**!
   * @return the shared point to the point cloud data (memory is reallocated)
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZCloudPtr() const;

  /**!
   * @return the median value of all points distances from sensor
   */
  float getMedianRange() const;

  /**!
   * Switches the axis from the KITTI convention
   */
  void setImageLikeAxisFromKitti();

  /**!
   * Switches the axis from the convention used on Toad robot of Robo@FIT group
   */
  void setImageLikeAxisFromBut();

  /**!
   * Loads the Velodyne point cloud from the KITTI *.bin file. The ring ID is also
   * estimated.
   *
   * @param infile input file
   * @param out_cloud [output] destination
   */
  static void fromKitti(const std::string &infile, VelodynePointCloud &out_cloud)
  {
    out_cloud.clear();
    // load point cloud
    std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
    if (!input.good())
    {
      std::cerr << "Could not read file: " << infile << std::endl;
      exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    int i;
    for (i = 0; input.good() && !input.eof(); i++)
    {
      velodyne_pointcloud::PointXYZIR point;
      input.read((char *)&point.x, 3 * sizeof(float));
      input.read((char *)&point.intensity, sizeof(float));
      out_cloud.push_back(point);
    }
    input.close();

    int ring = 0;
    int ring_size = out_cloud.size() / VELODYNE_RINGS_COUNT;
    for (int i = 0; i < out_cloud.size(); i++)
    {
      if (i != 0 && (i % ring_size) == 0)
      {
        ring++;
      }

      if (ring < VELODYNE_RINGS_COUNT)
      {
        out_cloud[i].ring = ring;
      }
      else
      {
        out_cloud.erase(out_cloud.begin() + i, out_cloud.end());
      }
    }
    out_cloud.setImageLikeAxisFromKitti();

    std::cerr << "Read KTTI point cloud " << infile << " with " << i << " points." << std::endl;
  }

protected:
  std::vector< std::vector<velodyne_pointcloud::PointXYZIR> > getRings() const;
  VelodynePointCloud discartWeakPoints(float threshold);

public:
  static const uint16_t VELODYNE_RINGS_COUNT = 64;      ///! Expected number of the rings in Velodyne point cloud
};

}

#endif /* VELODYNEPOINTCLOUD_H_ */
