 /*
 * Visual loop detection by VFH features.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 12/06/2015
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
#include <algorithm>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <cv.h>

#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_velodyne;

float CORRECT_LOOP_POSE_TOLERANCE = 3;   // 3m
int MINIMAL_TIME_OF_LOOP = 20;           // 10s
int VELODYNE_FPS = 10;

void check(bool condition, string what) {
  if(!condition) {
    cerr << "Check failed: " << what << endl;
    exit(1);
  }
}

void computeDistanceMatrix(const PointCloud<VFHSignature308>::Ptr &features, Mat &distances) {

  for(int row = 0; row < features->size(); row++) {
    Mat row_hist(1, 308, CV_32FC1, &(features->at(row).histogram));

    for(int col = 0; col < features->size(); col++) {
      Mat col_hist(1, 308, CV_32FC1, &(features->at(col).histogram));

      float distance;
      if(abs(row - col) < MINIMAL_TIME_OF_LOOP*VELODYNE_FPS) {
        distance = INFINITY;
      } else {
        distance = compareHist(row_hist, col_hist, CV_COMP_CHISQR);
      }
      distances.at<float>(row, col) = distance;
    }
  }
}

vector<DMatch> findMatches(const PointCloud<VFHSignature308>::Ptr &features) {
  Mat distances(features->size(), features->size(), CV_32F);
  computeDistanceMatrix(features, distances);

  vector<DMatch> matches;
  for(int row = 0; row < features->size(); row++) {
    double min_distance = INFINITY;
    Point min_position;
    minMaxLoc(distances.row(row), &min_distance, NULL, &min_position, NULL);

    if(min_position.x < 0 || min_position.x >= features->size()) {
      continue;
    }

    Point corresp_min;
    double corresp_min_dist = INFINITY;
    minMaxLoc(distances.row(min_position.x), &corresp_min_dist, NULL, &corresp_min, NULL);

    if((corresp_min.x == row) && (row < min_position.x)) {
      matches.push_back(DMatch(row, min_position.x, MIN(min_distance, corresp_min_dist)));
    }
  }

  return matches;
}

float posesDiff(const Eigen::Affine3f &pose1, const Eigen::Affine3f &pose2) {
  PointXYZ pt1 = KittiUtils::positionFromPose(pose1);
  PointXYZ pt2 = KittiUtils::positionFromPose(pose2);
  return euclideanDistance(pt1, pt2);
}

void printLoops(const vector<Eigen::Affine3f> &poses, vector<DMatch> matches,
                const vector<Eigen::Affine3f> &poses_gt) {

  int counter = 0;
  for(int i = 0; i < matches.size(); i++) {
    PointXYZ from = KittiUtils::positionFromPose(poses[matches[i].queryIdx]);
    PointXYZ to = KittiUtils::positionFromPose(poses[matches[i].trainIdx]);

    float feat_distance = sqrt(matches[i].distance);
    float space_distance = euclideanDistance(from, to);

    float distance_gt;
    if(poses_gt.empty()) {
      distance_gt = -1;
    } else {
      PointXYZ from_gt = KittiUtils::positionFromPose(poses_gt[matches[i].queryIdx]);
      PointXYZ to_gt = KittiUtils::positionFromPose(poses_gt[matches[i].trainIdx]);
      distance_gt = euclideanDistance(from_gt, to_gt);
    }

    cout << matches[i].trainIdx << " " << matches[i].queryIdx << " "
        << feat_distance << " " << space_distance << " " << distance_gt << endl;
  }
}

/**
 * for f in /media/files/kitti/data_odometry_velodyne/results/19-history-avg/evaluation/results/ivelas/data/*.txt; do i=$(basename $f | cut -d"." -f1); echo $i; ./find-loops /media/files/kitti/data_odometry_velodyne/results/19-history-avg/$i.txt /media/files/kitti/data_odometry_velodyne/dataset/vhf/$i-vfh.pcd; done
 */
int main(int argc, char** argv)
{
  if(argc != 4) {
    cerr << "Insufficient arguments. Usage: " << argv[0] <<
        " <vfh-feat.pcd> <poses-found> <poses-gt(maybe-invalid)>" << endl;
    return 1;
  } else {
    PointCloud<VFHSignature308>::Ptr features(new PointCloud<VFHSignature308>());
    io::loadPCDFile(argv[1], *features);
    vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[2]);
    vector<Eigen::Affine3f> poses_gt = KittiUtils::load_kitti_poses(argv[3], false);

    vector<DMatch> matches;
    matches = findMatches(features);
    printLoops(poses, matches, poses_gt);
  }

  return EXIT_SUCCESS;
}
