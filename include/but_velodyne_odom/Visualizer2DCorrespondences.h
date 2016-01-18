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

#ifndef VISUALISATION_H_
#define VISUALISATION_H_

#include <but_velodyne_odom/Correspondence.h>
#include <but_velodyne_odom/ImageLine.h>
#include <but_velodyne_odom/PointCloudLine.h>

namespace but_velodyne_odom {

class Visualizer2DCorrespondences {
public:
  Visualizer2DCorrespondences(
      const cv::Mat &source_image,
      const cv::Mat &target_image
  );

  void viewLineCorrespondences(
      const std::vector<ImageLine> &source_lines,
      const std::vector<ImageLine> &target_lines,
      const std::vector<cv::DMatch> &matches);

  bool view3DLineCorrenspondences(
      const std::vector<PointCloudLine> &source_lines,
      const std::vector<PointCloudLine> &target_lines,
      const std::vector<cv::DMatch> &matches,
      const cv::Mat &projection_matrix,
      const Eigen::Matrix4f &transformation
  );

protected:
  int show(std::string description);

  cv::Mat drawingImage;
  cv::Mat drawingImageLeftHalf, drawingImageRightHalf;
  cv::Rect imageFrame;
  cv::RNG& rng;
};

} /* namespace but_velodyne_odom */

#endif /* VISUALISATION_H_ */
