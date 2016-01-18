/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 07/11/2014
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

#ifndef IMAGELINEDETECTOR_H_
#define IMAGELINEDETECTOR_H_


#include <cv.h>
#include <velodyne_pointcloud/point_types.h>

namespace but_velodyne_odom {

class ImageLine {
public:
  cv::Point p1;
  cv::Point p2;
  float width, precision, nfa;

public:
  ImageLine(double *line_info) :
    p1(line_info[0], line_info[1]),
    p2(line_info[2], line_info[3]),
    width(line_info[4]),
    precision(line_info[5]),
    nfa(line_info[6]) {
  }

  ImageLine(cv::Point p1, cv::Point p2) :
    p1(p1), p2(p2),
    width(0), precision(0), nfa(0) {
  }

  Eigen::Vector2f orientation() const {
    Eigen::Vector2f orientation;
    orientation[0] = p2.x - p1.x;
    orientation[1] = p2.y - p1.y;
    return orientation;
  }

  // line: a*x + b*y + c = 0
  // return a*p.x + b*p.y + c
  inline float valueOfEquation(const cv::Point2f &point) const {
    Eigen::Vector2f orient = orientation();
    float a, b, c;
    a = orient[1]*(-1.0);
    b = orient[0];
    c = (a*p1.x + b*p1.y)*(-1.0);
    return a*point.x + b*point.y + c;
  }
};

}

#endif /* IMAGELINEDETECTOR_H_ */
