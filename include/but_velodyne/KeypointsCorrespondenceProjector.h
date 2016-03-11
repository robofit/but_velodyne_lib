/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 26/09/2014
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

#ifndef CORRESPONDENCEMERGE_H_
#define CORRESPONDENCEMERGE_H_

#include <but_velodyne/Correspondence.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Stopwatch.h>

namespace but_velodyne {

/**!
 * Re-projector of image correspondences to the 3D space.
 */
class KeypointsCorrespondenceProjector
{
public:

  /**!
   * @param _source_image previous image frame
   * @param _target_image current image frame
   * @param _source_cloud previous LiDAR point cloud frame
   * @param _target_cloud current LiDAR point cloud frame
   * @param projection_matrix intrinsic camera parameters
   * @param _images_correspondences input matches of image key-points
   */
  KeypointsCorrespondenceProjector(cv::Mat _source_image,
                          cv::Mat _target_image,
                          VelodynePointCloud _source_cloud,
                          VelodynePointCloud _target_cloud,
                          cv::Mat projection_matrix,
                          std::vector<Correspondence2D> _images_correspondences) :

                          source_image(_source_image),
                          target_image(_target_image),
                          source_cloud(_source_cloud),
                          target_cloud(_target_cloud),
                          images_correspondences(
                              _images_correspondences),
                          source_projection(
                              projectLidarData(_source_cloud,
                                               projection_matrix,
                                               _source_image)),
                          target_projection(
                              projectLidarData(_target_cloud,
                                               projection_matrix,
                                               _target_image))
  { }

  /**!
   * @return correspondences between source and target point cloud based on the matches of 2D key-points
   */
  std::vector<Correspondence3D> findLidarCorrespondences();

protected:
  const std::vector<Correspondence2D> images_correspondences;
  const std::vector<Correspondence3D2D> source_projection;
  const std::vector<Correspondence3D2D> target_projection;

  const static int DISTANCE_THRESHOLD = 5*5;    // px in L2

  // for debug purposes:
  const cv::Mat source_image, target_image;
  const VelodynePointCloud source_cloud, target_cloud;

  cv::Mat
  getTrainingPointsFromImageMatches(enum Direction direction);
  cv::Mat
  createQueryFromProjectedCloud(const std::vector<Correspondence3D2D> &projection);
  std::vector<Correspondence3D> mergeCorrespondences(
      const cv::Mat &source_indicies,
      const cv::Mat &source_distances,
      const cv::Mat &target_indicies,
      const cv::Mat &target_distances);

  static std::vector<Correspondence3D2D> projectLidarData(
      const VelodynePointCloud &point_cloud,
      const cv::Mat &projection_matrix,
      const cv::Mat &image)
  {
    using namespace std;
    using namespace cv;
    using namespace pcl;

    Stopwatch stopwatch;
    stopwatch.restart();

    //Mat imageToProject = image.clone();
    Rect frame(0, 0, image.cols, image.rows);
    vector<Correspondence3D2D> correspodences;
    for (VelodynePointCloud::const_iterator pt_3D = point_cloud.begin();
        pt_3D < point_cloud.end();
        pt_3D++)
    {
      Point2f projected_pt;
      if (projectPoint(*pt_3D, projection_matrix, frame, projected_pt))
      {
        correspodences.push_back(Correspondence3D2D(*pt_3D, projected_pt));
        //circle(imageToProject, projected_pt, 2, Scalar(0,0,pt_3D->intensity*255), -1);
      }
    }
    cerr << "Projection took: " << stopwatch.elapsed() << "[sec]" << endl;
    return correspodences;
  }
};

}

#endif /* CORRESPONDENCEMERGE_H_ */
