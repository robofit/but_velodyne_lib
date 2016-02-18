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

#ifndef _CORRESPONDENCE_H_
#define _CORRESPONDENCE_H_

#include <cv.h>
#include <velodyne_pointcloud/point_types.h>
#include <but_velodyne/PointCloudLine.h>

namespace but_velodyne {

/**!
 * Two ends of correspondence (a.k.a. arrow)
 */
enum Direction
{
  SOURCE, TARGET
};

/**!
 * General correspondence between two arbitrary elements.
 */
template<class PointTsrc, class PointTarget>
class Correspondence
{
public:
  PointTsrc source;
  PointTarget target;
  Correspondence(PointTsrc src, PointTarget target) :
      source(src), target(target), quality(1)
  {
  }
  float quality;
};

///! correspondence in image(s)
typedef Correspondence<cv::Point2f, cv::Point2f> Correspondence2D;

///! correspondence between point cloud element and image point
typedef Correspondence<velodyne_pointcloud::PointXYZIR,
                      cv::Point2f> Correspondence3D2D;

///! correspondence in point cloud(s)
typedef Correspondence<velodyne_pointcloud::PointXYZIR,
                      velodyne_pointcloud::PointXYZIR> Correspondence3D;

}

#endif /* CORRESPONDENCE_H_ */
