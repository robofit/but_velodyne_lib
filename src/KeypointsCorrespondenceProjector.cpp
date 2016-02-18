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

#include <vector>

#include <cv.h>
#include <highgui.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/KeypointsCorrespondenceProjector.h>
#include <but_velodyne/Correspondence.h>
#include <but_velodyne/Stopwatch.h>

using namespace std;
using namespace cv;


namespace but_velodyne {

void drawCorrespondence(Mat &img, Point2f projected, Point2f keypoint) {
  RNG& rng=theRNG();
  Scalar color(rng(256), rng(256), rng(256));
  circle(img, keypoint, 5, color, 1);
  circle(img, projected, 5, color, -1);
  line(img, keypoint, projected, color);
  float distance = powf(projected.x-keypoint.x, 2) + powf(projected.y-keypoint.y, 2);
}

void draw3DTo2DMatches(const Mat &image,
                       const vector<Correspondence3D2D> &projection,
                       const vector<Correspondence2D> &images_correspondences,
                       const enum Direction direction,
                       const Mat &indicies) {
  Mat drawing_image = image.clone();
  for(int i = 0; i < projection.size(); i++) {
    Point2f projected_pt = projection[i].target;
    Correspondence2D correspodence = images_correspondences[indicies.at<int>(i)];
    Point2f key_pt = (direction == SOURCE) ? correspodence.source : correspodence.target;
    drawCorrespondence(drawing_image, projected_pt, key_pt);
  }
  stringstream image_filename;
  image_filename << "2Dto3D_merged_correspodences_" << direction << ".png";
  imwrite(image_filename.str(), drawing_image);
}

vector<Correspondence3D> KeypointsCorrespondenceProjector::findLidarCorrespondences()
{
  Stopwatch stopwatch;
  stopwatch.restart();

  cv::flann::KDTreeIndexParams index_params(1);
  Mat source_image_keypoints = getTrainingPointsFromImageMatches(SOURCE);
  cv::flann::Index source_kdTree(source_image_keypoints, index_params);
  Mat target_image_keypoints = getTrainingPointsFromImageMatches(TARGET);
  cv::flann::Index target_kdTree(target_image_keypoints, index_params);

  Mat source_query = createQueryFromProjectedCloud(source_projection);
  Mat source_indicies(source_query.rows, 1, CV_32SC1);
  Mat source_distances(source_query.rows, 1, CV_32FC1);
  source_kdTree.knnSearch(source_query, source_indicies, source_distances, 1);

  /*draw3DTo2DMatches(source_image,
                    source_projection,
                    images_correspondences,
                    SOURCE,
                    source_indicies);*/

  Mat target_query = createQueryFromProjectedCloud(target_projection);
  Mat target_indicies(target_query.rows, 1, CV_32SC1);
  Mat target_distances(target_query.rows, 1, CV_32FC1);
  target_kdTree.knnSearch(target_query, target_indicies, target_distances, 1);

  /*draw3DTo2DMatches(target_image,
                    target_projection,
                    images_correspondences,
                    TARGET,
                    target_indicies);*/

  cerr << "Nearest projected points found in: " << stopwatch.elapsed() << "[sec]" << endl;
  stopwatch.restart();
  vector<Correspondence3D> final_correspondences = mergeCorrespondences(source_indicies,
                                                                       source_distances,
                                                                       target_indicies,
                                                                       target_distances);
  cerr << "Merged in: " << stopwatch.elapsed() << "[sec]" << endl;
  cerr << "Correspondences projected: " << final_correspondences.size();
  return final_correspondences;
}

Mat KeypointsCorrespondenceProjector::getTrainingPointsFromImageMatches(enum Direction direction)
{
  Mat points_2D(images_correspondences.size(), 2, CV_32F);
  int index = 0;
  for (vector<Correspondence2D>::const_iterator coresp = images_correspondences.begin();
      coresp < images_correspondences.end(); coresp++, index++)
  {
    Point2f pt = (direction == SOURCE) ? coresp->source : coresp->target;
    points_2D.at<float>(index, 0) = pt.x;
    points_2D.at<float>(index, 1) = pt.y;
  }
  return points_2D;
}

Mat KeypointsCorrespondenceProjector::createQueryFromProjectedCloud(const vector<Correspondence3D2D> &projection)
{
  Mat query(projection.size(), 2, CV_32F);
  for (int i = 0; i < projection.size(); i++)
  {
    query.at<float>(i, 0) = projection[i].target.x;
    query.at<float>(i, 1) = projection[i].target.y;
  }
  return query;
}

vector<Correspondence3D> KeypointsCorrespondenceProjector::mergeCorrespondences(const Mat &source_indicies,
                                                                      const Mat &source_distances,
                                                                      const Mat &target_indicies,
                                                                      const Mat &target_distances)
{
  vector<Correspondence3D> result;
  Mat source_draw_img = source_image.clone();
  Mat target_draw_img = target_image.clone();

  for (int source_i = 0; source_i < source_projection.size(); source_i++)
  {
    if (source_distances.at<float>(source_i) < DISTANCE_THRESHOLD)
    {

      for (int target_i = 0; target_i < target_projection.size(); target_i++)
      {
        if (target_distances.at<float>(target_i) < DISTANCE_THRESHOLD)
        {

          if (source_indicies.at<int>(source_i) == target_indicies.at<int>(target_i))
          {
            // just debug:
            Correspondence2D corresp_2D_source = images_correspondences[source_indicies.at<int>(source_i)];
            Correspondence2D corresp_2D_target = images_correspondences[target_indicies.at<int>(target_i)];
            drawCorrespondence(source_draw_img, source_projection[source_i].target, corresp_2D_source.source);
            drawCorrespondence(target_draw_img, target_projection[target_i].target, corresp_2D_target.target);
            /*cerr << "source_dist: " << source_distances.at<float>(source_i) << "\t"
                 << "target_dist: " << target_distances.at<float>(target_i) << endl;
            imshow("Source", source_draw_img);
            waitKey(50);
            imshow("Target", target_draw_img);
            waitKey(0);
            source_draw_img = source_image.clone();
            target_draw_img = target_image.clone();*/

            result.push_back(Correspondence3D(source_projection[source_i].source, target_projection[target_i].source));
            result.back().quality = 1 / (source_distances.at<float>(source_i) + target_distances.at<float>(target_i));
          }
        }
      }

    }
  }

  imwrite("merged_2D_corresp_source.png", source_draw_img);
  imwrite("merged_2D_corresp_target.png", target_draw_img);
  return result;
}

}

