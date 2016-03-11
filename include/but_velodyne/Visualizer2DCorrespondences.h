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

#include <but_velodyne/Correspondence.h>
#include <but_velodyne/ImageLine.h>
#include <but_velodyne/PointCloudLine.h>

namespace but_velodyne {

/**!
 * Visualization of the corresponding lines in 2D.
 */
class Visualizer2DCorrespondences {
public:

  /**!
   * @param source_image source image used as a background for visualization
   * @param target_image target image used as a background for visualization
   */
  Visualizer2DCorrespondences(
      const cv::Mat &source_image,
      const cv::Mat &target_image
  );

  /**!
   * Show the correspondences among 2D lines.
   *
   * @param source_lines lines from source image
   * @param target_lines lines from target image
   * @param matches correspondences already found among 2D lines
   */
  void viewLineCorrespondences(
      const std::vector<ImageLine> &source_lines,
      const std::vector<ImageLine> &target_lines,
      const std::vector<cv::DMatch> &matches);

  /**!
   * Show the correspondences among 3D lines on the 2D plane.
   *
   * @param source_lines lines from the source observation
   * @param target_lines lines from the target observation
   * @param matches correspondences already found among 3D lines
   * @param projection_matrix 3x4 projection matrix (intrinsic and extrinsic camera parameters)
   * @param transformation 3D transformation from source to target coordinate system
   */
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

} /* namespace but_velodyne */

#endif /* VISUALISATION_H_ */
