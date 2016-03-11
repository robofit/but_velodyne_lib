/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 23/01/2015
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

#ifndef VISUALISER3D_H_
#define VISUALISER3D_H_

#include <iostream>

#include <cv.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/Correspondence.h>

using namespace std;

namespace but_velodyne
{

/**!
 * Wrapper of PCL visualizer for displaying 3D data. Visualizer follows
 * builder design pattern. Used should add everything (s)he wants to visualize
 * including metadata (colors, ...) and then trigger the visualization
 * calling show() method.
 */
class Visualizer3D
{
public:
  Visualizer3D();

  ~Visualizer3D();

  /**!
   * Prints rotation and translation coefficients of [R|t] transformation
   *
   * @param transformation [R|t] transformation
   */
  static void printRT(const Eigen::Matrix4f &transformation) {
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f affineTransf(transformation);
    pcl::getTranslationAndEulerAngles(affineTransf, x, y, z, roll, pitch, yaw);
    cerr << "t: [" << x << ", " << y << ", " << z << "]\t" <<
        "R: [" << roll << ", " << pitch << ", " << yaw << "]" << endl;
  }

  /**!
   * Add new point cloud of arbitrary point type into the visualization. Cloud
   * is optionally transformed. The color for whole cloud is randomly generated
   * or the user defined color is used if was previously set.
   *
   * @param cloud point cloud to visualized
   * @param transformation optional 3D transformation of the cloud before the visualization
   * @return *this instance with the new point cloud ready to be visualized too
   */
  template<typename PointT>
  Visualizer3D& addPointCloud(const pcl::PointCloud<PointT> cloud,
                              const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity()) {
    pcl::PointCloud<PointT> cloud_transformed;
    transformPointCloud(cloud, cloud_transformed, transformation);
    if(!transformation.isIdentity()) {
      cerr << "Transformation:" << endl << transformation.matrix() << endl;
      printRT(transformation);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    unsigned rgb[] = { rngU(), rngU(), rngU()} ;

    for (typename pcl::PointCloud<PointT>::const_iterator pt = cloud_transformed.begin();
        pt < cloud_transformed.end(); pt++)
    {
      pcl::PointXYZRGB color_pt(rgb[0], rgb[1], rgb[2]);
      color_pt.x = pt->x;
      color_pt.y = pt->y;
      color_pt.z = pt->z;
      color_cloud->push_back(color_pt);
    }
    return addColorPointCloud(color_cloud);
  }

  /**!
   * Add new point cloud of XYZRGB point type into the visualization. Cloud
   * is optionally transformed. The original color of all points is preserved.
   *
   * @param cloud point cloud to visualized
   * @param transformation optional 3D transformation of the cloud before the visualization
   * @return *this instance with the new point cloud ready to be visualized too
   */
  Visualizer3D& addColorPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity());

  /**!
   * Add new point clouds of arbitrary point type into the visualization. The color for
   * whole cloud is randomly generated or the user defined color is used if was previously set.
   *
   * @param clouds point clouds to visualized
   * @return *this instance with the new point clouds ready to be visualized too
   */
  template<typename PointT>
  Visualizer3D& addPointClouds(const std::vector< pcl::PointCloud<PointT> > &clouds) {
    for(std::vector<VelodynePointCloud>::const_iterator cloud = clouds.begin();
        cloud < clouds.end(); cloud++) {
      addPointCloud(*cloud);
    }
    return *this;
  }

  /**!
   * Add line segments bound by the corresponding 3D points. The color for each line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param correspondences corresponding 3D points (endpoints of the line segments)
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const std::vector<Correspondence3D> &correspondences);

  /**!
   * Add line segments into the visualization. The color for each line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param lines new lines to be added
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const std::vector<PointCloudLine> &lines);

  /**!
   * Add line segments into the visualization using specific color.
   *
   * @param lines new lines to be added
   * @param r red channel of the color used for visualization
   * @param g green channel of the color used for visualization
   * @param b blue channel of the color used for visualization
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const std::vector<PointCloudLine> &lines,float r, float g, float b);

  /**!
   * Add line segments of the cloud into the visualization. The color for each line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param lineCloud new lines of line cloud to be added
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLines(const LineCloud &lineCloud);

  /**!
   * Add line segment bound by the corresponding 3D points. The color for the line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param line corresponding 3D point (endpoint of the line segment)
   * @return *this instance with line segment added for visualization
   */
  Visualizer3D& addLine(const Correspondence3D &line);

  /**!
   * Add line segment into the visualization. The color for the line
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param line new line to be added
   * @return *this instance with line segment added for visualization
   */
  Visualizer3D& addLine(const PointCloudLine &line);

  /**!
   * Add line segment into the visualization using specific color.
   *
   * @param line new line to be added
   * @param r red channel of the color used for visualization
   * @param g green channel of the color used for visualization
   * @param b blue channel of the color used for visualization
   * @return *this instance with line segments added for visualization
   */
  Visualizer3D& addLine(const PointCloudLine &line, float r, float g, float b);

  /**!
   * Add the arrow into the visualization. The color for the arrow
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param arrow new arrow to be added
   * @return *this instance with arrow added for visualization
   */
  Visualizer3D& addArrow(const PointCloudLine &arrow);

  /**!
   * Add the sensor representation into the visualization (currently the yellow sphere)
   *
   * @param position position of the sensor
   * @return *this instance with sensor representation added
   */
  Visualizer3D& addSenzor(pcl::PointXYZ position = pcl::PointXYZ(0, 0, 0));

  /**!
   * Add the visualization of 3D matches. The color for each match
   * is randomly generated or the user defined color is used if was previously set.
   *
   * @param matches matches to be added
   * @param source_pts source points of matches (indexed by trainIdx)
   * @param target_pts target points of matches (indexed by queryIdx)
   * @return *this instance with matches added for visualization
   */
  template<typename PointT>
  Visualizer3D& addMatches(const std::vector<cv::DMatch> &matches,
                           const pcl::PointCloud<PointT> source_pts,
                           const pcl::PointCloud<PointT> target_pts) {
    int i = 0;
    for(std::vector<cv::DMatch>::const_iterator m = matches.begin();
        m < matches.end(); m++) {
      assert(m->trainIdx < source_pts.size() && m->queryIdx < target_pts.size());
      assert(m->trainIdx >= 0 && m->queryIdx >= 0);
      viewer->addArrow(source_pts[m->trainIdx], target_pts[m->queryIdx],
                       rngU(), rngU(), rngU(), false, getId("arrow"));
    }
    return *this;
  }

  /**!
   * Add visualization of 3D poses (one point per pose)
   *
   * @param poses poses to visualize (sequence of 3D poses of sensor/vehicle expected)
   * @return *this instance with poses added for visualization
   */
  Visualizer3D& addPosesDots(const vector<Eigen::Affine3f> &poses);

  /**!
   * Add visualization of visual loops between 3D poses
   *
   * @param poses sequence of 3D poses of sensor/vehicle expected
   * @param visual loops detected
   * @return *this instance with visual loops added for visualization
   */
  Visualizer3D& addPosesLoops(const vector<Eigen::Affine3f> &poses,
                              cv::vector<cv::DMatch> matches = cv::vector<cv::DMatch>());

  /**!
   * Shows the interactive visualization of the all elements added.
   */
  void show() {
    viewer->spin();
  }

  /**!
   * Shows the single frame of visualization of the all elements added.
   *
   * @param time duration of visualization in miliseconds
   */
  void showOnce(int time = 1) {
    viewer->spinOnce(time);
  }

  /**!
   * Saves the single frame of visualization of the all elements added into the image file.
   *
   * @param filename destination image file
   */
  void saveSnapshot(const std::string &filename);

  /**!
   * Close the visualization.
   * !!! There is a bug on Ubuntu-like systems and visualization can not be stopped.
   * This can result in SIGSEGV when multiple visualizations are launched. !!!
   */
  void close() {
    viewer->close();
  }

  /**!
   * Discards all point clouds from visualization except the last N clouds.
   *
   * @param count number of clouds preserved in the visualization
   * @return *this instance with point clouds discarded
   */
  Visualizer3D& keepOnlyClouds(int count);

  /**!
   * Set the color for visualization of the next element (or set of elements in case of point clouds).
   *
   * @param r red channel of the color used for next visualization
   * @param g green channel of the color used for next visualization
   * @param b blue channel of the color used for next visualization
   */
  Visualizer3D& setColor(unsigned r, unsigned g, unsigned b);

  /**!
   * @return the encapsulated PCLVisualizer instance for specific setups.
   */
  const boost::shared_ptr<pcl::visualization::PCLVisualizer>& getViewer() const
  {
    return viewer;
  }

protected:
  std::string getId(const string &what);

  double rngF();

  unsigned rngU();

protected:
  cv::RNG& rng;
  int color_index;
  vector<unsigned> color_stack;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  int identifier;
  vector<string> all_identifiers;
};

} /* namespace but_velodyne */

#endif /* VISUALISER3D_H_ */
