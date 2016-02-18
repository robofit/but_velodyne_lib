/*
 * Conversion from SLAM++ solution to KITTI format.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * With code snippets from SLAM++ prject: http://sourceforge.net/p/slam-plus-plus/wiki/Home/
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 20/07/2015
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

#include <cstdlib>
#include <cstdio>

#include <but_velodyne/KittiUtils.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

/**
 * @brief converts axis-angle representation to quaternion
 * @tparam Derived is Eigen derived matrix type for the first matrix argument
 * @param[in] r_axis_angle is a vector where unit direction gives axis, and magnitude gives angle in radians
 * @param[out] r_quat is quaternion representing the same rotation as r_axis_angle
 * @return Returns rotation angle in radians (after flushing small angles to zero).
 */
template<class Derived>
static double AxisAngle_to_Quat(const Eigen::MatrixBase<Derived> &r_axis_angle,
		Eigen::Quaterniond &r_quat) {

	double f_angle = r_axis_angle.norm();
	if (f_angle < 1e-12) { // increasing this does not help
		r_quat = Eigen::Quaterniond(1, 0, 0, 0); // cos(0) = 1
		return 0; //M_PI * 2;
	} else {
		double f_half_angle = fmod(f_angle, M_PI * 2) * .5;
		double q = sin(f_half_angle) / f_angle; // sin is [0, 1], angle is about [-2pi, 2pi] but could be slightly more / less due to optimization incrementing / decrementing it
		r_quat = Eigen::Quaterniond(cos(f_half_angle), r_axis_angle(0) * q,
				r_axis_angle(1) * q, r_axis_angle(2) * q);
		r_quat.normalize(); // should already be normalized unles there was a great roundoff in calculation of q; could detect that.
	}
	return f_angle;
}

/**
 * @brief converts from axis angle representation to rotation matrix
 * @tparam Derived is Eigen derived matrix type for the first matrix argument
 * @param[in] v_vec is axis-angle rotation (angle is encoded as the magnitude of the axis)
 * @return Returns rotation matrix, corresponding to the input.
 * @note Consider whether converting to a quaternion would not be faster: converting to a quaternion
 * 	and transforming a single point will be faster than converting to a rotation matrix. Only when
 *  transforming two or more points, rotation matrix will be faster.
 */
template<class Derived>
Eigen::Matrix3d AxisAngle_to_RotMatrix(const Eigen::MatrixBase<Derived> &v_vec) {
	Eigen::Quaternion<double> rot;
	AxisAngle_to_Quat(v_vec, rot);
	return rot.toRotationMatrix();
}


int main(int argc, char** argv)
{
  vector<Eigen::Affine3f> poses;

  while(true) {
    float x, y, z;
    Eigen::Vector3f r_axis_angle;
    cin >> x >> y >> z >> r_axis_angle(0) >> r_axis_angle(1) >> r_axis_angle(2);

    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.matrix().block(0, 0, 3, 3) = AxisAngle_to_RotMatrix(r_axis_angle).cast<float>();
    pose.matrix()(0, 3) = x;
    pose.matrix()(1, 3) = y;
    pose.matrix()(2, 3) = z;

    if(cin.eof()) {
      break;
    } else {
      poses.push_back(pose);
    }
  }

  KittiUtils::save_kitti_poses(poses, cout);

  return EXIT_SUCCESS;
}
