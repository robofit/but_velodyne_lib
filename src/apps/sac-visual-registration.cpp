/*
 * Visual keypoints based registration of Velodyne point clouds.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 20/08/2014
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

#include <string>

#include <velodyne_pointcloud/point_types.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <but_velodyne/Correspondence.h>
#include <but_velodyne/KeypointsCorrespondenceProjector.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer2DCorrespondences.h>
#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace cv;
using namespace but_velodyne;

// obtained via manual-calibration application (considered static camera-Velodyne setup)
const Eigen::Affine3f CALIBRATION = getTransformation(-0.04, -0.19, -0.18, 0.04, 0.02, -0.035);

std::vector<Correspondence2D> findImageCorrespondences(
    const cv::Mat &source_image,
    const cv::Mat &target_image)
{
  using namespace cv;
  using namespace std;

  ORB feat_detector;
  vector<KeyPoint> source_keypoints, target_keypoints;
  feat_detector.detect(source_image, source_keypoints);
  feat_detector.detect(target_image, target_keypoints);

  Ptr<DescriptorExtractor> feat_extractor = DescriptorExtractor::create("ORB");
  Mat source_descriptors, target_descriptors;
  feat_extractor->compute(source_image, source_keypoints, source_descriptors);
  feat_extractor->compute(target_image, target_keypoints, target_descriptors);

  BFMatcher matcher;
  vector<DMatch> matches;
  matcher.match(target_descriptors, source_descriptors, matches);

/*  Mat matches_image;
  drawMatches(target_image, target_keypoints, source_image, source_keypoints, matches, matches_image);
  imwrite("matches.png", matches_image);
  waitKey(100);*/

  vector<Correspondence2D> pairs2D;
  for (vector<DMatch>::iterator match = matches.begin(); match < matches.end(); match++)
  {
    Point2f source_pt = source_keypoints[match->trainIdx].pt;
    Point2f target_pt = target_keypoints[match->queryIdx].pt;
    pairs2D.push_back(Correspondence2D(source_pt, target_pt));
  }
  return pairs2D;
}

void splitCorrespondences(const vector<Correspondence3D> &correnspondences,
                          PointCloud<PointXYZ>::Ptr source,
                          PointCloud<PointXYZ>::Ptr target,
                          pcl::CorrespondencesPtr index_corrensp) {
  unsigned i = 0;
  for(vector<Correspondence3D>::const_iterator correnspondence = correnspondences.begin();
      correnspondence < correnspondences.end();
      correnspondence++, i++) {
    source->push_back(PointXYZIRtoPointXYZ(correnspondence->source));
    target->push_back(PointXYZIRtoPointXYZ(correnspondence->target));
    index_corrensp->push_back(pcl::Correspondence(i, i, 1/correnspondence->quality));
  }
}

int main(int argc, char *argv[])
{
	if(argc != 3) {
	    cerr << "Insufficient arguments. Usage: " << argv[0] << "<data-folder-1> <data-folder-2>";
	    return EXIT_FAILURE;
	}

	string data_path_1 = argv[1];
	string data_path_2 = argv[2];

  VelodynePointCloud source_cloud, target_cloud;
  Mat source_image, target_image;
  io::loadPCDFile(data_path_1, source_cloud);
  transformPointCloud(source_cloud, source_cloud, CALIBRATION);
  io::loadPCDFile(data_path_2, target_cloud);
  transformPointCloud(target_cloud, target_cloud, CALIBRATION);

  source_image = imread(data_path_1 + "/camera_image.png");
  target_image = imread(data_path_2 + "/camera_image.png");
  Rect frame(0, 0, source_image.cols, source_image.rows);

  FileStorage projection_storrage(data_path_1 + "/projection.yml", FileStorage::READ);
  Mat projection_matrix;
  projection_storrage["P"] >> projection_matrix;

  /* ******************************************************************** */

  Stopwatch stopwatch;

  vector<Correspondence2D> image_correspondences =
      findImageCorrespondences(source_image, target_image);
  cerr << "Image correspondences:\t" << image_correspondences.size() << endl;
  cerr << "Image corresp.:\t" << stopwatch.elapsed() << "[sec]" << endl;

  KeypointsCorrespondenceProjector projector(source_image, target_image,
                                    source_cloud, target_cloud,
                                    projection_matrix,
                                    image_correspondences);
  cerr << "Projector cereated:\t" << stopwatch.elapsed() << "[sec]" << endl;
  vector<Correspondence3D> clouds_correspodences = projector.findLidarCorrespondences();
  cerr << "Cloud corresp.:\t" << stopwatch.elapsed() << "[sec]" << endl;

  registration::CorrespondenceRejectorSampleConsensus<PointXYZ> rejector;
  PointCloud<PointXYZ>::Ptr source_key_cloud(new PointCloud<PointXYZ>());
  PointCloud<PointXYZ>::Ptr target_key_cloud(new PointCloud<PointXYZ>());
  CorrespondencesPtr index_correnspondences(new Correspondences());
  splitCorrespondences(clouds_correspodences,
                       source_key_cloud,
                       target_key_cloud,
                       index_correnspondences);
  rejector.setInputSource(source_key_cloud);
  rejector.setInputTarget(target_key_cloud);
  rejector.setInputCorrespondences(index_correnspondences);

  cerr << "Preparation done:\t" << stopwatch.elapsed() << "[sec]" << endl;

  Correspondences inlier_correspondences;
  rejector.getCorrespondences(inlier_correspondences);
  cerr << "Inliers count: " << inlier_correspondences.size() << endl;

  cerr << "Elapsed total:\t" << stopwatch.elapsed() << "[sec]" << endl;

  Visualizer3D().addPointCloud(source_cloud).
      addPointCloud(target_cloud, rejector.getBestTransformation()).show();

  return 0;
}
