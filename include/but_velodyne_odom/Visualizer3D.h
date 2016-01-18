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

#include <but_velodyne_odom/VelodynePointCloud.h>
#include <but_velodyne_odom/LineCloud.h>
#include <but_velodyne_odom/Correspondence.h>

using namespace std;

namespace but_velodyne_odom
{

class Visualizer3D
{
public:
  Visualizer3D();

  ~Visualizer3D();

  static void printRT(const Eigen::Matrix4f &transformation) {
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f affineTransf(transformation);
    pcl::getTranslationAndEulerAngles(affineTransf, x, y, z, roll, pitch, yaw);
    cerr << "t: [" << x << ", " << y << ", " << z << "]\t" <<
        "R: [" << roll << ", " << pitch << ", " << yaw << "]" << endl;
  }

  template<typename PointT>
  Visualizer3D& addPointCloud(const pcl::PointCloud<PointT> cloud,
                              const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity()) {
    pcl::PointCloud<PointT> cloud_transformed;
    transformPointCloud(cloud, cloud_transformed, transformation);
    if(transformation.isIdentity()) {
      //cerr << "Transformation: Identity" << endl;
    } else {
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

  Visualizer3D& addColorPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity()) {

    pcl::transformPointCloud(*cloud, *cloud, transformation);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_vis(cloud);
    std::string id = getId("cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb_vis, id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2, id);
    return *this;
  }

  template<typename PointT>
  Visualizer3D& addPointClouds(const std::vector< pcl::PointCloud<PointT> > &clouds) {
    for(std::vector<VelodynePointCloud>::const_iterator cloud = clouds.begin();
        cloud < clouds.end(); cloud++) {
      addPointCloud(*cloud);
    }
    return *this;
  }

  Visualizer3D& addLines(const std::vector<Correspondence3D> &correspondences);
  Visualizer3D& addLines(const std::vector<PointCloudLine> &lines);
  Visualizer3D& addLines(const std::vector<PointCloudLine> &lines,float r, float g, float b);
  Visualizer3D& addLines(const LineCloud &lineCloud);
  Visualizer3D& addLine(const Correspondence3D &line);
  Visualizer3D& addLine(const PointCloudLine &line);
  Visualizer3D& addLine(const PointCloudLine &line, float r, float g, float b);
  Visualizer3D& addArrow(const PointCloudLine &line);
  Visualizer3D& addSenzor(pcl::PointXYZ position = pcl::PointXYZ(0, 0, 0));

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

  Visualizer3D& addPosesLoops(const vector<Eigen::Affine3f> &poses,
                              cv::vector<cv::DMatch> matches = cv::vector<cv::DMatch>());

  Visualizer3D& addPosesDots(const vector<Eigen::Affine3f> &poses);

  void show() {
    viewer->spin();
  }

  void showOnce(int time = 1) {
    viewer->spinOnce(time);
  }

  Visualizer3D& saveSnapshot(const std::string &filename);

  void close() {
    viewer->close();
  }

  Visualizer3D& keepOnlyClouds(int count);

  Visualizer3D& setColor(unsigned r, unsigned g, unsigned b) {
    color_stack.push_back(r);
    color_stack.push_back(g);
    color_stack.push_back(b);
    return *this;
  }

  const boost::shared_ptr<pcl::visualization::PCLVisualizer>& getViewer() const
  {
    return viewer;
  }

protected:
  std::string getId(const string &what) {
    std::stringstream ss;
    ss << what << "_" << identifier++;
    all_identifiers.push_back(ss.str());
    return ss.str();
  }

  double rngF() {
    return rng.uniform(0.0, 1.0);
  }

  unsigned rngU() {
    if(color_stack.size() > color_index) {
      return color_stack[color_index++];
    } else {
      return rng(256);
    }
  }

protected:
  cv::RNG& rng;
  int color_index;
  vector<unsigned> color_stack;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  int identifier;
  vector<string> all_identifiers;
};

} /* namespace but_velodyne_odom */

#endif /* VISUALISER3D_H_ */
