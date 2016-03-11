/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 10/10/2014
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

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cv.h>
#include <highgui.h>

#include <but_velodyne/Visualizer2DCorrespondences.h>

using namespace pcl;
using namespace cv;
using namespace std;
using namespace velodyne_pointcloud;

namespace but_velodyne {

Visualizer2DCorrespondences::Visualizer2DCorrespondences(
    const cv::Mat &source_image,
    const cv::Mat &target_image) :
        imageFrame(0, 0, source_image.cols, source_image.rows),
        rng(theRNG()),
        drawingImage(source_image.rows, source_image.cols+target_image.cols, CV_8UC3) {

  drawingImageLeftHalf  = drawingImage.colRange(0, source_image.cols);
  drawingImageRightHalf = drawingImage.colRange(source_image.cols,
                                                source_image.cols+target_image.cols);
  Mat source_color_image, target_color_image;
  cvtColor(source_image, source_color_image, CV_GRAY2RGB);
  cvtColor(target_image, target_color_image, CV_GRAY2RGB);

  source_color_image.copyTo(drawingImageLeftHalf);
  target_color_image.copyTo(drawingImageRightHalf);
}


void Visualizer2DCorrespondences::viewLineCorrespondences(
                                           const vector<ImageLine> &source_lines,
                                           const vector<ImageLine> &target_lines,
                                           const vector<DMatch> &matches) {
  for(vector<DMatch>::const_iterator match = matches.begin();
      match < matches.end();
      match++) {
    Scalar color(rng(256), rng(256), rng(256));
    line(drawingImageLeftHalf, source_lines[match->trainIdx].p1, source_lines[match->trainIdx].p2,
         color, 5);
    line(drawingImageRightHalf, target_lines[match->queryIdx].p1, target_lines[match->queryIdx].p2,
         color, 5);

    Point target_start_point = target_lines[match->queryIdx].p1;
    target_start_point.x += drawingImageLeftHalf.cols;
    line(drawingImage, source_lines[match->trainIdx].p1, target_start_point,
         color, 2);
  }

  show("Matched lines");
}

bool Visualizer2DCorrespondences::view3DLineCorrenspondences(
                           const vector<PointCloudLine> &source_lines,
                           const vector<PointCloudLine> &target_lines,
                           const vector<DMatch> &matches,
                           const Mat &projection_matrix,
                           const Eigen::Matrix4f &transformation) {
  for(vector<DMatch>::const_iterator match = matches.begin();
        match < matches.end();
        match++) {
    PointCloudLine source_line = source_lines[match->trainIdx];
    PointCloudLine target_line = target_lines[match->queryIdx];

    Scalar color(rng(256), rng(256), rng(256));

    ImageLine source_img_line = source_line.project(projection_matrix, imageFrame);
    ImageLine target_img_line = target_line.project(projection_matrix, imageFrame);

    float t_distance = source_line.distanceTo(target_line.transform(transformation),
                                              PointCloudLine::EUCLIDEAN);
    float r_distance = source_line.distanceTo(target_line.transform(transformation),
                                              PointCloudLine::COSINE_ORIENTATION);

    line(drawingImageLeftHalf,  source_img_line.p1, source_img_line.p2, color, 5);
    line(drawingImageRightHalf, target_img_line.p1, target_img_line.p2, color, 5);

    stringstream ss;
    ss << "t:" << t_distance << ";r:" << r_distance;
    Point text_pt = source_img_line.p1 + Point(0,-10);
    putText(drawingImageLeftHalf, ss.str(), text_pt, FONT_HERSHEY_PLAIN, 1.0, color);

    target_img_line.p1.x += drawingImageLeftHalf.cols;
    line(drawingImage, source_img_line.p1, target_img_line.p1, color, 2);
  }
  cerr << "Viewing " << matches.size() << " matches." << endl;
  return show("matched filtered lines") == 27;
}

int Visualizer2DCorrespondences::show(std::string description) {
  imshow(description, drawingImage);
  char key = waitKey(0);
  imwrite(description + ".png", drawingImage);
  return key;
}

}
