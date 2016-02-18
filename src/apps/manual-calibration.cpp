/*
 * Manual calibration of Velodyne LiDAR with RGB camera.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2014
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

#include <cv.h>
#include <highgui.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_velodyne/VelodynePointCloud.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

class CommonException : public exception {
public:
  CommonException(string msg) : msg(msg) {
  }
  virtual const char* what() const throw() {
    return string("Exception '" + msg + "' occurred.").c_str();
  }
  ~CommonException() throw() {};
private:
  string msg;
};

class ManualCalibration {
public:
  ManualCalibration(string directory_name) :
    x_t(0), y_t(0), z_t(0), x_r(0), y_r(0), z_r(0),
    finished(false), last_key(27), diff_t(0.01), diff_r(0.005) {

    image = imread(directory_name + "/camera_image.png", CV_LOAD_IMAGE_COLOR);
    if(image.data == NULL) {
      throw CommonException("image can't be read");
    }

    if(io::loadPCDFile(directory_name + "/velodyne_pc.pcd", point_cloud) < 0) {
      throw CommonException("point-cloud can't be read");
    }

    FileStorage projection_storrage(directory_name + "/projection.yml", FileStorage::READ);
    if(!projection_storrage.isOpened()) {
      throw CommonException("storage of the projection matrix was not opened");
    }
    projection_storrage["P"] >> projection_matrix;
    cerr << "projection matrix: " << endl << projection_matrix << endl << endl;
  }

  bool isFinished() {
    return finished;
  }

  void show() {
    Mat projection = image.clone();
    Rect frame(0, 0, image.cols, image.rows);

    Eigen::Affine3f calibration = getTransformation(x_t, y_t, z_t, x_r, y_r, z_r);
    cerr << "calibration matrix:" << endl << calibration.matrix() << endl;
    VelodynePointCloud transformed_cloud;
    transformPointCloud(point_cloud, transformed_cloud, calibration);

    for(VelodynePointCloud::iterator pt_3D = transformed_cloud.begin(); pt_3D < transformed_cloud.end(); pt_3D++) {
      Point2f pt_2D;
      if(projectPoint(*pt_3D, projection_matrix, frame, pt_2D)) {
        circle(projection, pt_2D, 2, Scalar(0,0,pt_3D->intensity*10), -1);
      }
      Point3f pt_3D_cv(pt_3D->x, pt_3D->y, pt_3D->z);
    }
    imshow("Fused image with pointcloud", projection);
    last_key = waitKey(0);
  }

  void reloadParameters() {
    switch(last_key) {
      case -83: // -
        factorDiff(0.5); break;
      case -85: // +
        factorDiff(2.0); break;

      case 'q':
        x_t += diff_t; break;
      case 'a':
        x_t -= diff_t; break;

      case 'w':
        y_r -= diff_r; break;
      case 's':
        y_r += diff_r; break;

      case 'e':
        y_t += diff_t; break;
      case 'd':
        y_t -= diff_t; break;

      case 'r':
        x_r -= diff_r; break;
      case 'f':
        x_r += diff_r; break;

      case 't':
        z_t += diff_t; break;
      case 'g':
        z_t -= diff_t; break;

      case 'z':
        z_r -= diff_r; break;
      case 'h':
        z_r += diff_r; break;

      case 27:
        finished = true;
        break;

      default:
        cerr << last_key << " (unknown): " << (int)last_key << endl;
    }

    printCalibration();
  }

  void printCalibration() {
    cout << "[" << x_t << ", " << y_t << ", " << z_t << ", "
                << x_r << ", " << y_r << ", " << z_r << "]" << endl;
  }
private:

  VelodynePointCloud getVisiblePoints(const VelodynePointCloud &input) {
    VelodynePointCloud output;
    Point2f trash;
    Rect frame(0, 0, image.cols, image.rows);
    for(VelodynePointCloud::const_iterator pt_3D = input.begin();
        pt_3D < input.end();
        pt_3D++) {
      if(projectPoint(*pt_3D, projection_matrix, frame, trash)) {
        output.push_back(*pt_3D);
      }
    }
    return output;
  }

  void factorDiff(float f) {
    diff_t *= f;
    diff_r *= f;

    cerr << "diff_t: " << diff_t << "\t diff_r:" << diff_r << endl;
  }

  Mat image;
  VelodynePointCloud point_cloud;
  Mat projection_matrix;
  float x_t, y_t, z_t, x_r, y_r, z_r;

  bool finished;
  char last_key;
  float diff_t, diff_r;
};

int main(int argc, char *argv[]) {
  if(argc < 2) {
    throw CommonException("missing argument: <data-folder>");
  }

  ManualCalibration calibration(argv[1]);
  while(!calibration.isFinished()) {
    calibration.show();
    calibration.reloadParameters();
  }

  return EXIT_SUCCESS;
}
