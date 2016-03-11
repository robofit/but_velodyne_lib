/*
 * Verification of detected visual loops.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 02/07/2015
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
#include <sstream>
#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PoseGraphEdge.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_velodyne;

// experimentally set thresholds
float MAX_FEAT_DISTANCE = 3;
float MAX_SPACE_DISTANCE = 20;

void check(bool condition, string what) {
  if(!condition) {
    cerr << "Check failed: " << what << endl;
    exit(1);
  }
}

cv::vector<cv::DMatch> loadMatches(const string &matches_filename) {
  std::ifstream file(matches_filename.c_str());
  if(!file.is_open()) {
    std::perror((std::string("Unable to open file: ") + matches_filename).c_str());
    exit(1);
  }

  cv::vector<cv::DMatch> matches;
  while(true) {
    int train, query;
    float feat_dist, space_dist, gt_dist;
    file >> train >> query >> feat_dist >> space_dist >> gt_dist;

    if(file.eof()) {
      break;
    } else {
      if(feat_dist < MAX_FEAT_DISTANCE && space_dist < MAX_SPACE_DISTANCE) {
        matches.push_back(cv::DMatch(query, train, feat_dist));
      }
    }
  }

  cerr << "Loaded " << matches.size() << "\t matches from " << matches_filename << endl;
  return matches;
}

int main(int argc, char *argv[])
{
  if(argc == 4) {
    Visualizer3D visualizer;
    vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[1]);
    vector<Eigen::Affine3f> poses_closed = KittiUtils::load_kitti_poses(argv[2]);
    vector<Eigen::Affine3f> poses_gt = KittiUtils::load_kitti_poses(argv[3]);

    visualizer.setColor(255, 0, 0).addPosesDots(poses_gt).show();
    visualizer.setColor(0, 255, 0).addPosesDots(poses_closed);
    visualizer.setColor(0, 0, 255).addPosesDots(poses);

    visualizer.show();

  } else {
    cerr << "Insufficient arguments. Usage: " << argv[0] <<
        " <poses> <poses-closed> <poses-gt>" << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
