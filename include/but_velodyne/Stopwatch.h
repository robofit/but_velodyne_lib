/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 24/10/2014
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

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <ctime>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>

namespace but_velodyne {

/**!
 * Simple time measurement.
 */
class Stopwatch {
public:

  /**!
   * Also starts the stopwatch
   */
  Stopwatch() {
    restart();
  }

  /**!
   * Restarts completely the time measurement
   */
  void restart() {
    start_time = boost::posix_time::microsec_clock::local_time();
    already_elapsed = boost::posix_time::time_duration();
    paused = false;
  }

  /**!
   * Just pause the measurement - the elapsed time is kept.
   */
  void pause() {
    if (!paused) {
      paused = true;
      already_elapsed += boost::posix_time::microsec_clock::local_time() - start_time;
    }
  }

  /**!
   * Continue the measurement
   */
  void goOn() {
    if (paused) {
      paused = false;
      start_time = boost::posix_time::microsec_clock::local_time();
    }
  }

  /**!
   * Stopwatch are not paused, stopped or reset - measurement continues.
   *
   * @return the time elapsed so far
   */
  double elapsed() {
    return (boost::posix_time::microsec_clock::local_time() - start_time + already_elapsed).total_microseconds() / 1e6;
  }

private:
  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration already_elapsed;
  bool paused;
};

}

#endif /* STOPWATCH_H_ */

