/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/01/2015
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

#ifndef RANSAC_H_
#define RANSAC_H_

#include <cv.h>
#include <algorithm>

namespace but_velodyne_odom
{

template <class TModel, class TData>
class Ransac
{
public:
  Ransac(const cv::TermCriteria termination, const std::vector<TData> &data,
         int min_inliers) :
    termination(termination), data(data),
    best_inliers(data.size(), 0),
    min_inliers(min_inliers) {
  }

  TModel run();

  std::vector<int> getInliersMask() {
    return best_inliers;
  }

protected:
  std::vector<TData> getSubset(const std::vector<int> &mask);
  std::vector<TData> getRandomSubset(int count);

  bool isFinished(int iterations, float min_error) {
    if (termination.type == cv::TermCriteria::MAX_ITER) {
      return iterations > termination.maxCount;
    } else if(termination.type == cv::TermCriteria::EPS) {
      return min_error < termination.epsilon;
    } else {
      return (iterations > termination.maxCount) || (min_error < termination.epsilon);
    }
  }

protected:
  const cv::TermCriteria termination;
  const std::vector<TData> data;

  std::vector<int> best_inliers;

  const int min_inliers;        // minimal number of inliers for feasible model
};

template <class TModel, class TData>
TModel Ransac<TModel, TData>::run() {
  assert(data.size() >= TModel::minimalDataCount());

  std::cerr << "TModel Ransac<TModel, TData>::run()" << std::endl;

  int iterations = 0;
  TModel best_model;
  float min_error = INFINITY;

  for(int iterations = 0; !isFinished(iterations, min_error); iterations++) {
    std::vector<TData> min_subset = getRandomSubset(TModel::minimalDataCount());
    TModel model(min_subset);

    int num_of_inliers = 0;
    std::vector<int> inliers_mask;
    for(int i = 0; i < data.size(); i++) {
      if(model.fits(data[i])) {
        inliers_mask.push_back(1);
        num_of_inliers++;
      } else {
        inliers_mask.push_back(0);
      }
    }

    if(num_of_inliers > min_inliers) {  // feasible model
      std::vector<TData> inliers = getSubset(inliers_mask);
      model = TModel(inliers);
      float error = model.computeError(inliers);
      if(error < min_error) {
        best_model = model;
        min_error = error;
        best_inliers = inliers_mask;
      }
    }
    std::cerr << iterations << "\t" << num_of_inliers << "\t" << min_error << std::endl;
  }
  return best_model;
}


template <class TModel, class TData>
std::vector<TData> Ransac<TModel, TData>::getRandomSubset(int count) {
  std::vector<int> mask(data.size(), 0);
  for(int i = 0; i < count; i++) {
    mask[i] = 1;
  }
  std::random_shuffle(mask.begin(), mask.end());
  return getSubset(mask);
}

template <class TModel, class TData>
std::vector<TData> Ransac<TModel, TData>::getSubset(const std::vector<int> &mask) {
  std::vector<TData> subset;
  for(int i = 0; i < mask.size(); i++) {
    if(mask[i] > 0) {
      subset.push_back(data[i]);
    }
  }
  return subset;
}


} /* namespace but_velodyne_odom */

#endif /* RANSAC_H_ */
