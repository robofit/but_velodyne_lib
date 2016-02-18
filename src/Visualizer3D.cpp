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

#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

namespace but_velodyne
{

using namespace cv;
using namespace pcl;
using namespace Eigen;

Visualizer3D::Visualizer3D() :
    rng(cv::theRNG()),
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    identifier(0),
    color_index(0)
{
  viewer->setBackgroundColor(255, 255, 255);
  viewer->addCoordinateSystem(0.5);
  viewer->initCameraParameters();
  viewer->setCameraPosition(5, -500, 0, 0, 0, 0);
}

Visualizer3D::~Visualizer3D() {
  close();
}

Visualizer3D& Visualizer3D::addColorPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const Eigen::Matrix4f &transformation) {

  pcl::transformPointCloud(*cloud, *cloud, transformation);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_vis(cloud);
  std::string id = getId("cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb_vis, id);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                           2, id);
  return *this;
}

Visualizer3D& Visualizer3D::addLine(const Correspondence3D &line) {
  viewer->addLine(line.source, line.target, rngF(), rngF(), rngF(), getId("line"));
  return *this;
}

Visualizer3D& Visualizer3D::addLine(const PointCloudLine &line) {
  return addLine(line, rngF(), rngF(), rngF());
}

Visualizer3D& Visualizer3D::addLine(const PointCloudLine &line,
                                    float r, float g, float b) {
  viewer->addLine(line.getBeginPoint(), line.getEndPoint(),
                  r, g, b, getId("line"));
  return *this;
}

Visualizer3D& Visualizer3D::addArrow(const PointCloudLine &line) {
  this->addLine(line);
  viewer->addSphere(line.getEndPoint(), 0.2, rngF(), rngF(), rngF(), getId("sphere"));
/*  viewer->addArrow(line.getBeginPoint(), line.getEndPoint(),
                    rng(256), rng(256), rng(256), false, "arrow_" + getId());
                    */
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const std::vector<Correspondence3D> &lines) {
  for (vector<Correspondence3D>::const_iterator l = lines.begin(); l < lines.end(); l++)
  {
    addLine(*l);
  }
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const std::vector<PointCloudLine> &lines,
                                     float r, float g, float b) {
  for (vector<PointCloudLine>::const_iterator l = lines.begin(); l < lines.end(); l++)
  {
    addLine(*l, r, g, b);
  }
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const std::vector<PointCloudLine> &lines) {
  for (vector<PointCloudLine>::const_iterator l = lines.begin(); l < lines.end(); l++)
  {
    addLine(*l);
  }
  return *this;
}

Visualizer3D& Visualizer3D::addLines(const LineCloud &lineCloud) {
  return addLines(lineCloud.getLines());
}

void Visualizer3D::saveSnapshot(const std::string &filename) {
  viewer->saveScreenshot(filename);
}

Visualizer3D& Visualizer3D::addSenzor(PointXYZ position) {
  string name = "senzor";
  float radius = 1.5;
  viewer->removeShape(name);
  position.y = -2 * radius;
  viewer->addSphere(position, 3, 1.0, 1.0, 0, name);
  return *this;
}

Visualizer3D& Visualizer3D::keepOnlyClouds(int clouds_to_preserve) {
  vector<string> old_ids = all_identifiers;
  all_identifiers.clear();
  int preserved = 0;
  for(int i = old_ids.size() - 1; i >= 0; i--) {
    string id = old_ids[i];
    if(id.find("cloud") != string::npos) {
      if(preserved < clouds_to_preserve) {
        preserved++;
        all_identifiers.push_back(id);
      } else {
        viewer->removePointCloud(id);
      }
    } else {
      all_identifiers.push_back(id);
    }
  }
  reverse(all_identifiers.begin(), all_identifiers.end());
  return *this;
}

Visualizer3D& Visualizer3D::addPosesLoops(const vector<Eigen::Affine3f> &poses,
                                 cv::vector<cv::DMatch> matches) {
  PointXYZ to(0, 0, 0);
  for(int i = 1; i < poses.size(); i++) {
    PointXYZ from = to;
    to = KittiUtils::positionFromPose(poses[i]);
    this->addLine(PointCloudLine(from, to), .7, .7, .7);
  }

  for(int i = 0; i < matches.size(); i++) {
    PointXYZ from = KittiUtils::positionFromPose(poses[matches[i].queryIdx]);
    PointXYZ to = KittiUtils::positionFromPose(poses[matches[i].trainIdx]);
    this->addArrow(PointCloudLine(from, to));
  }

  return *this;
}

Visualizer3D& Visualizer3D::addPosesDots(const vector<Eigen::Affine3f> &poses) {
  PointCloud<PointXYZ> poses_cloud;
  for(vector<Eigen::Affine3f>::const_iterator p = poses.begin(); p < poses.end(); p++) {
    poses_cloud.push_back(KittiUtils::positionFromPose(*p));
  }
  return addPointCloud(poses_cloud);
}

Visualizer3D& Visualizer3D::setColor(unsigned r, unsigned g, unsigned b) {
  color_stack.push_back(r);
  color_stack.push_back(g);
  color_stack.push_back(b);
  return *this;
}

std::string Visualizer3D::getId(const string &what) {
  std::stringstream ss;
  ss << what << "_" << identifier++;
  all_identifiers.push_back(ss.str());
  return ss.str();
}

double Visualizer3D::rngF() {
  return rng.uniform(0.0, 1.0);
}

unsigned Visualizer3D::rngU() {
  if(color_stack.size() > color_index) {
    return color_stack[color_index++];
  } else {
    return rng(256);
  }
}

} /* namespace but_velodyne */
