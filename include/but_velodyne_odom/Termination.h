/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 22/05/2015
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

#ifndef TERMINATION_H_
#define TERMINATION_H_

#include <boost/circular_buffer.hpp>
#include <boost/range/numeric.hpp>

#include <but_velodyne_odom/Stopwatch.h>

namespace but_velodyne_odom
{

class ErrorDeviation {
public:
  ErrorDeviation(int iterations) :
    last_errors(iterations) {
  }

  void add(float error) {
    last_errors.push_back(error);
  }

  float getDeviation() {
    float sum_squares = 0;
    float mean = getMean();
    for(boost::circular_buffer<float>::iterator e = last_errors.begin();
        e < last_errors.end(); e++) {
      sum_squares += pow(*e - mean, 2);
    }
    return sqrt(sum_squares / last_errors.size());
  }

  float getMean() {
    return std::accumulate(last_errors.begin(), last_errors.end(), 0.0) / last_errors.size();
  }

  bool isSignificant(float threshold) {
    return (last_errors.size() != last_errors.capacity()) ||
        (getDeviation() > threshold);
  }
private:
  boost::circular_buffer<float> last_errors;
};


class Termination
{
public:
  Termination(int min_iterations, int max_iterations, float max_time_spent,
              float min_err_deviation, float min_error);

  void addNewError(float error);

  bool operator()();
private:
  Stopwatch stopwatch;
  ErrorDeviation err_deviation;
  const float min_error;
  const float max_time_spent;
  const float max_iterations;
  const float min_err_deviation;

  float last_error;
  int iterations;
};

} /* namespace but_velodyne_odom */

#endif /* TERMINATION_H_ */
