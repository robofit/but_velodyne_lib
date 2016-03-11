/*
 * Extraction of VFH features for Velodyne point cloud.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 16/06/2015
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

#include <cstdlib>
#include <cstdio>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

void downsample(const PointCloud<PointXYZ>::Ptr cloud,
                const PointCloud<PointXYZ>::Ptr downsampled,
                int factor) {
  for(int i = 0; i < cloud->size(); i++) {
    if(i % factor == 0) {
      downsampled->push_back(cloud->at(i));
    }
  }
}

VFHSignature308 getVFHistigram(const PointCloud<PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<PointXYZ>());
  downsample(cloud, downsampled_cloud, 10);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (downsampled_cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normals (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree_normals);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.2);
  ne.compute (*cloud_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_vfh(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud(downsampled_cloud);
  vfh.setInputNormals(cloud_normals);
  vfh.setSearchMethod(tree_vfh);

  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
  vfh.compute(*vfhs);

  return vfhs->front();
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << "<output-feat.pcd> <point-cloud>+" << endl;
    return EXIT_FAILURE;
  } else {
    PointCloud<VFHSignature308>::Ptr features(new PointCloud<VFHSignature308>());
    VelodynePointCloud cloud;
    for(int i = 2; i < argc; i++) {
      string kitti_scan = argv[i];
      cerr << "scan: " << kitti_scan << endl;
      VelodynePointCloud::fromKitti(kitti_scan, cloud);

      VFHSignature308 vfh = getVFHistigram(cloud.getXYZCloudPtr());
      features->push_back(vfh);
    }
    cerr << "Extracted " << features->size() << " features." << endl;
    io::savePCDFile(argv[1], *features);
  }

  return EXIT_SUCCESS;
}
