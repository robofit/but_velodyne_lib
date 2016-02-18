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

#include <but_velodyne/Stopwatch.h>

namespace but_velodyne
{

/**!
 * Standard deviation of the registration error.
 */
class ErrorDeviation {
public:

  /**!
   * @param iterations how many iterations should be considered
   */
  ErrorDeviation(int iterations) :
    last_errors(iterations) {
  }

  /**!
   * Adds the new error value after the iteration of registration algorithm
   *
   * @param error new error value
   */
  void add(float error) {
    last_errors.push_back(error);
  }

  /**!
   * @return the standard deviation of the error in last N iterations
   */
  float getDeviation() const;

  /**!
   * @return the mean error of last N iterations
   */
  float getMean() const;

  /**!
   * @return true off the error deviation is over the threshold value
   */
  bool isSignificant(float threshold) {
    return (last_errors.size() != last_errors.capacity()) ||
        (getDeviation() > threshold);
  }
private:
  boost::circular_buffer<float> last_errors;
};

/**!
 * Criteria for termination of the iterative algorithm.
 * Algorithm is terminated when one of the following criteria is met:
 *  - algorithm exceeded maximal number of iterations
 *  - algorithm exceeded time resources given
 *  - standard deviation of the error yielded from last N iterations is insignificant
 *  - error from the last iteration is bellow the threshold
 */
class Termination
{
public:

  /**!
   * @param min_iterations minimal number of the iterations
   * @param max_iterations maximum iterations
   * @param max_time_spent maximum time resources for the algorithm
   * @param min_err_deviation minimal standard deviation of the error allowed (computed from multiple iterations of algorithm)
   * @param min_error minimal algorithm error
   */
  Termination(int min_iterations, int max_iterations, float max_time_spent,
              float min_err_deviation, float min_error);

  /**!
   * Add the error from the last algorithm iteration.
   */
  void addNewError(float error);

  /**!
   * @return true if algorithm should be terminated
   */
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

} /* namespace but_velodyne */

#endif /* TERMINATION_H_ */
