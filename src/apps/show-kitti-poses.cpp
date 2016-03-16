/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
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
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

void toColor(uchar i, uchar &r, uchar &g, uchar &b) {
  if(i < 128) {
    b = 2*i;
    r = g = 0;
  } else {
    b = 255;
    r = g = (i-128)*2;
  }
}

void addVelodynePcl(Visualizer3D &vis, const VelodynePointCloud &cloud, const Eigen::Affine3f &pose) {
  PointCloud<PointXYZRGB>::Ptr rgb_cloud(new PointCloud<PointXYZRGB>());

  float min = cloud.getMinValuePt().intensity;
  float max = cloud.getMaxValuePt().intensity;

  for(VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    uchar r, g, b;
    float normalized = (pt->intensity - min) / (max - min) * 255.0;
    toColor(MIN(normalized*2, 255), r, g, b);
    PointXYZRGB rgb_pt;
    rgb_pt.x = pt->x;
    rgb_pt.y = pt->y;
    rgb_pt.z = pt->z;
    rgb_pt.r = r;
    rgb_pt.g = g;
    rgb_pt.b = b;
    rgb_cloud->push_back(rgb_pt);
  }

  vis.addColorPointCloud(rgb_cloud, pose.matrix());
}


int main(int argc, char** argv)
{
  if(argc < 2) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << " <poses> <point-cloud>+";
    return 1;
  }

  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[1]);

  Visualizer3D visualizer;
  VelodynePointCloud cloud;

  PointXYZ senzor(0,0,0);
  for(int i = 0; i < argc-2; i++) {
    string kitti_scan = argv[i+2];
    cerr << "scan: " << kitti_scan << endl;
    //VelodynePointCloud::fromKitti(kitti_scan, cloud);
    if (kitti_scan.find(".pcd") != string::npos) {
        pcl::io::loadPCDFile(kitti_scan, cloud);
    } else {
      VelodynePointCloud::fromKitti(kitti_scan, cloud);
    }

    addVelodynePcl(visualizer, cloud, poses[i]);
  }
  visualizer.show();

  return EXIT_SUCCESS;
}
