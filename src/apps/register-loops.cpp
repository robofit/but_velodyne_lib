/*
 * Registration of Velodyne scans where visual loop was found.
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
#include <algorithm>
#include <sstream>
#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/PoseGraphEdge.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_velodyne;

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
//      if(feat_dist < 3 && space_dist < 20) {
      if(feat_dist < 1.68 && space_dist < 5) {
        matches.push_back(cv::DMatch(query, train, feat_dist));
      }
    }
  }

  cerr << "Loaded " << matches.size() << "\t matches from " << matches_filename << endl;
  return matches;
}

// DEPRECATED
Eigen::Matrix4f getRotationByPoses(const Eigen::Affine3f &source_t, const Eigen::Affine3f &target_t) {

  Eigen::Matrix4f source_r = Eigen::Matrix4f::Identity();
  source_r.block(0, 0, 3 ,3) = source_t.rotation().matrix();

  cerr << "source_r: " << endl << source_r << endl;
  Visualizer3D::printRT(source_r); cerr << endl;

  Eigen::Matrix4f target_r = Eigen::Matrix4f::Identity();
  target_r.block(0, 0, 3 ,3) = target_t.rotation().matrix();
  Eigen::Matrix4f target_r_inv = target_r.inverse();

  cerr << "target_r: " << endl << target_r << endl;
  Visualizer3D::printRT(target_r); cerr << endl;
  cerr << "target_r(inv): " << endl << target_r_inv << endl;
  Visualizer3D::printRT(target_r_inv); cerr << endl;

  return (target_r_inv*source_r).inverse();
}

void print(ostream &out_file, const vector<PoseGraphEdge> &poseGraphEdges) {
  for(vector<PoseGraphEdge>::const_iterator p = poseGraphEdges.begin();
      p < poseGraphEdges.end(); p++) {
    out_file << *p << endl;
  }
}

void save2D(const VelodynePointCloud &cloud, const string &filename) {
  float radius = 50;
  float px_per_m = 10;
  float img_size = radius*2*px_per_m;
  Mat projection = Mat::zeros(img_size, img_size, CV_8UC1);
  Rect frame(0, 0, img_size, img_size);
  for(VelodynePointCloud::const_iterator p = cloud.begin(); p < cloud.end(); p++) {
    Point pt;
    pt.x = (p->x+radius) * px_per_m;
    pt.y = (p->z+radius) * px_per_m;
    if(pt.inside(frame)) {
      projection.at<uchar>(pt) = 255;
    }
  }
  imwrite(filename, projection);
}

vector<Eigen::Matrix4f> load_transformations(const string &transf_dir,
                                             const int poses_count) {
  vector<Eigen::Matrix4f> transformations(poses_count);
  for(int i = 0; i < poses_count; i++) {
    EigenUtils::loadMatrix(transf_dir + "/" + KittiUtils::getKittiFrameName(i) + ".transform",
                           transformations[i]);
  }
  return transformations;
}

/**
 * f for f in /media/files/kitti/data_odometry_velodyne/results/19-history-avg/evaluation/results/ivelas/data/*.txt; do i=$(basename $f | cut -d"." -f1); ./register-loops /media/files/kitti/data_odometry_velodyne/dataset/loop-matches/$i-matches.txt /media/files/kitti/data_odometry_velodyne/results/19-history-avg/$i.txt /media/files/kitti/data_odometry_velodyne/dataset/sequences/$i/velodyne/; done
 */
int main(int argc, char** argv)
{
  if(argc != 7) {
    cerr << "Insufficient arguments. Usage: " << argv[0] <<
        " <matches> <poses> <covariances> <kitti-seq-dir> <transf-seq-dir> <output-graph-file>" <<
        endl;
    return 1;
  } else {

    string matches_file = argv[1];
    vector<cv::DMatch> matches = loadMatches(matches_file);
    vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[2]);
    string covariances_file = argv[3];
    string seq_dir = argv[4];
    string transf_dir = argv[5];
    vector<Eigen::Matrix4f> transformations = load_transformations(transf_dir, poses.size());
    string out_file = argv[6];

    Visualizer3D().addPosesLoops(poses, matches).saveSnapshot(matches_file + ".png");

    vector<PoseGraphEdge> poseGraphEdges;

    vector<Mat> covariances;
    FileStorage cov_fs(covariances_file, FileStorage::READ);
    cov_fs["covariances"] >> covariances;
    for(int i = 1; i < poses.size(); i++) {
      if(sum(covariances[i]) != Scalar(0)) {
        poseGraphEdges.push_back(PoseGraphEdge(i-1, i, transformations[i], covariances[i]));
      } else {
        poseGraphEdges.push_back(PoseGraphEdge(i-1, i, transformations[i]));
      }
    }

    for(vector<cv::DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
      VelodynePointCloud source, target;
      VelodynePointCloud::fromKitti(seq_dir + "/" + KittiUtils::getKittiFrameName(m->queryIdx),
                                    source);
      VelodynePointCloud::fromKitti(seq_dir + "/" + KittiUtils::getKittiFrameName(m->trainIdx),
                                    target);

      //Visualizer3D().addPointCloud(source).addPointCloud(target).show();

      Eigen::Matrix4f init_rotation = getRotationByPoses(poses[m->queryIdx],
                                                         poses[m->trainIdx]);
      //Visualizer3D().addPointCloud(source)
      //    .addPointCloud(target, init_rotation).show();

      transformPointCloud(target, target, init_rotation);

      CollarLinesRegistration::Parameters registration_parameters(
          CollarLinesRegistration::MEDIAN_THRESHOLD,
          CollarLinesRegistration::VERTICAL_ANGLE_WEIGHTS
      );
      CollarLinesRegistrationPipeline::Parameters pipeline_parameters(
          10,   // generated
          2,    // preserved
          LineCloud::NONE,
          201,  // min iterations
          2000, // max iterations
          20,   // max time spent
          50    // iterations per sampling
      );

      LinearMoveEstimator linear_estimator(3);
      CollarLinesRegistrationPipeline registration(linear_estimator, cerr,
                                                   pipeline_parameters, registration_parameters);
      Mat covariance;
      registration.runRegistration(source, covariance);
      Eigen::Matrix4f t = registration.runRegistration(target, covariance);     // only zeros
      //Visualizer3D().addPointCloud(source).addPointCloud(target, t).show();

      cout << "final transformation: " << init_rotation*t << endl;

      poseGraphEdges.push_back(PoseGraphEdge(m->queryIdx, m->trainIdx,
                                             init_rotation*t));
      //showLoops(poses, vector<DMatch>(1, *m));
    }

    ofstream graphFile(out_file.c_str());
    print(graphFile, poseGraphEdges);
  }

  return EXIT_SUCCESS;
}
