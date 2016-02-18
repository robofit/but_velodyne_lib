/*
 * show_correspondences.cpp
 *
 *  Created on: 22.1.2016
 *      Author: ivelas
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

#include <cv.h>

#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace but_velodyne;
using namespace cv;

typedef double dgc_transform_t[4][4];

void dgc_warning(const char *fmt, ...)
{
  char message[1024];
  va_list args;

  va_start(args, fmt);
  vsnprintf(message, 1024, fmt, args);
  va_end(args);
  message[1023] = '\0';

  fprintf(stderr, "%s %s\n", "# WARNING: ", message);
}

inline char *dgc_next_word(char *str)
{
  char *mark = str;

  if(str == NULL)
    return NULL;
  while(*mark != '\0' && !(*mark == ' ' || *mark == '\t')) {
    mark++;
  }
  while(*mark != '\0' &&  (*mark == ' ' || *mark == '\t')) {
    mark++;
  }
  return mark;
}

static inline double dgc_d2r(double theta)
{
  return (theta * M_PI / 180.0);
}

void dgc_transform_identity(dgc_transform_t t)
{
  int r, c;

  for(r = 0; r < 4; r++)
    for(c = 0; c < 4; c++)
      if(r == c)
        t[r][c] = 1;
      else
        t[r][c] = 0;
}

void dgc_transform_left_multiply(dgc_transform_t t1, dgc_transform_t t2)
{
  dgc_transform_t result;
  int i, j, k;

  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++) {
      result[i][j] = 0;
      for(k = 0; k < 4; k++)
        result[i][j] += t2[i][k] * t1[k][j];
    }
  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++)
      t1[i][j] = result[i][j];
}

void dgc_transform_multiply(dgc_transform_t t1_dst, dgc_transform_t t2)
{
  dgc_transform_t result;
  int i, j, k;

  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++) {
      result[i][j] = 0;
      for(k = 0; k < 4; k++)
        result[i][j] += t1_dst[i][k] * t2[k][j];
    }
  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++)
      t1_dst[i][j] = result[i][j];
}

void dgc_transform_rotate_x(dgc_transform_t t, double theta)
{
  dgc_transform_t temp;
  double ctheta = cos(theta), stheta = sin(theta);

  dgc_transform_identity(temp);
  temp[1][1] = ctheta;
  temp[1][2] = -stheta;
  temp[2][1] = stheta;
  temp[2][2] = ctheta;
  dgc_transform_left_multiply(t, temp);
}

void dgc_transform_rotate_y(dgc_transform_t t, double theta)
{
  dgc_transform_t temp;
  double ctheta = cos(theta), stheta = sin(theta);

  dgc_transform_identity(temp);
  temp[0][0] = ctheta;
  temp[0][2] = stheta;
  temp[2][0] = -stheta;
  temp[2][2] = ctheta;
  dgc_transform_left_multiply(t, temp);
}

void dgc_transform_rotate_z(dgc_transform_t t, double theta)
{
  dgc_transform_t temp;
  double ctheta = cos(theta), stheta = sin(theta);

  dgc_transform_identity(temp);
  temp[0][0] = ctheta;
  temp[0][1] = -stheta;
  temp[1][0] = stheta;
  temp[1][1] = ctheta;
  dgc_transform_left_multiply(t, temp);
}

void dgc_transform_translate(dgc_transform_t t, double x, double y, double z)
{
  t[0][3] += x;
  t[1][3] += y;
  t[2][3] += z;
}

void dgc_transform_copy(dgc_transform_t dest, dgc_transform_t src)
{
  int r, c;

  for(r = 0; r < 4; r++)
    for(c = 0; c < 4; c++)
      dest[r][c] = src[r][c];
}

int dgc_transform_read(dgc_transform_t t, const char *filename)
{
  FILE *fp;
  char *err, *mark, *unit, line[1001];
  double arg, x, y, z;

  /* start with identity transform */
  dgc_transform_identity(t);
  fp = fopen(filename, "r");
  if(fp == NULL) {
    dgc_warning("Error: could not open transform file %s.\n", filename);
    return -1;
  }
  do {
    err = fgets(line, 1000, fp);
    if(err != NULL) {
      unit = dgc_next_word(line);
      mark = dgc_next_word(unit);
      if(strncasecmp(line, "rx ", 3) == 0) {
        arg = strtod(mark, &mark);
        if(strncasecmp(unit, "deg", 3) == 0)
          arg = dgc_d2r(arg);
        dgc_transform_rotate_x(t, arg);
      }
      else if(strncasecmp(line, "ry ", 3) == 0) {
        arg = strtod(mark, &mark);
        if(strncasecmp(unit, "deg", 3) == 0)
          arg = dgc_d2r(arg);
        dgc_transform_rotate_y(t, arg);
      }
      else if(strncasecmp(line, "rz ", 3) == 0) {
        arg = strtod(mark, &mark);
        if(strncasecmp(unit, "deg", 3) == 0)
          arg = dgc_d2r(arg);
        dgc_transform_rotate_z(t, arg);
      }
      else if(strncasecmp(line, "t ", 2) == 0) {
        char *a = strdup("test");
        x = strtod(mark, &mark);
        y = strtod(mark, &mark);
        z = strtod(mark, &mark);

        if(strncasecmp(unit, "in", 2) == 0) {
          x *= 0.0254;
          y *= 0.0254;
          z *= 0.0254;
        }
        else if(strncasecmp(unit, "cm", 2) == 0) {
          x *= 0.01;
          y *= 0.01;
          z *= 0.01;
        }
        dgc_transform_translate(t, x, y, z);
      }
      else {
        dgc_warning("Error: could not parse line \"%s\" from %s\n",
                    line, filename);
        return -1;
      }

    }
  } while(err != NULL);
  fclose(fp);
  return 0;
}

void dgc_transform_get_translation(dgc_transform_t t, double *x, double *y,
                                   double *z)
{
  *x = t[0][3];
  *y = t[1][3];
  *z = t[2][3];
}

void dgc_transform_get_rotation(dgc_transform_t t, double *x, double *y,
                                double *z)
{
  *x = atan2(t[2][1], t[2][2]);
  *y = asin(-t[2][0]);
  *z = atan2(t[1][0], t[0][0]);
}

void dgc_transform_write(dgc_transform_t t, FILE *fp)
{
  double x, y, z;
  double rx, ry, rz;

  dgc_transform_get_rotation(t, &rx, &ry, &rz);
  dgc_transform_get_translation(t, &x, &y, &z);

  fprintf(fp,"RX RAD %lf\n", rx);
  fprintf(fp,"RY RAD %lf\n", ry);
  fprintf(fp,"RZ RAD %lf\n", rz);
  fprintf(fp,"T M %lf %lf %lf\n", x, y, z);
}

int dgc_transform_write(dgc_transform_t t, const char *filename)
{
  FILE *fp = fopen(filename, "w");
  if(fp == NULL) {
    dgc_warning("Error: could not open transform file %s for writing.\n", filename);
    return -1;
  }

  dgc_transform_write(t, fp);

  fclose(fp);
  return 0;
}

void dgc_transform(dgc_transform_t rot,
                       double tx, double ty, double tz,
                       double roll, double pitch, double yaw)
{
  double sinroll = sin(roll), cosroll  = cos(roll);
  double sinpitch = sin(pitch), cospitch = cos(pitch);
  double sinyaw = sin(yaw), cosyaw = cos(yaw);

  /* construct rotation matrix by hand */
  rot[0][0] = cosyaw * cospitch;
  rot[0][1] = cosyaw * sinpitch * sinroll - sinyaw * cosroll;
  rot[0][2] = cosyaw * sinpitch * cosroll + sinyaw * sinroll;
  rot[0][3] = tx;
  rot[1][0] = sinyaw * cospitch;
  rot[1][1] = sinyaw * sinpitch * sinroll + cosyaw * cosroll;
  rot[1][2] = sinyaw * sinpitch * cosroll - cosyaw * sinroll;
  rot[1][3] = ty;
  rot[2][0] = -sinpitch;
  rot[2][1] = cospitch * sinroll;
  rot[2][2] = cospitch * cosroll;
  rot[2][3] = tz;
  rot[3][0] = 0;
  rot[3][1] = 0;
  rot[3][2] = 0;
  rot[3][3] = 1;
}

void dgc_transform_inverse(dgc_transform_t in, dgc_transform_t out)
{
  double temp, t1, t2, t3;

  dgc_transform_copy(out, in);

  temp = out[0][1];
  out[0][1] = out[1][0];
  out[1][0] = temp;

  temp = out[0][2];
  out[0][2] = out[2][0];
  out[2][0] = temp;

  temp = out[1][2];
  out[1][2] = out[2][1];
  out[2][1] = temp;

  t1 =
    -out[0][0] * out[0][3]
    -out[0][1] * out[1][3]
    -out[0][2] * out[2][3];
  t2 =
    -out[1][0] * out[0][3]
    -out[1][1] * out[1][3]
    -out[1][2] * out[2][3];
  t3 =
    -out[2][0] * out[0][3]
    -out[2][1] * out[1][3]
    -out[2][2] * out[2][3];

  out[0][3] = t1;
  out[1][3] = t2;
  out[2][3] = t3;
}

void load_transformation(const string &filename, Eigen::Matrix4f &mpose) {
	static dgc_transform_t to_kitti_t = {{0, -1,  0,  0},
	                                     {0,  0, -1,  0},
	                                     {1,  0,  0,  0},
	                                     {0,  0,  0,  1}};

	dgc_transform_t to_kitti_t_inv;
	dgc_transform_inverse(to_kitti_t, to_kitti_t_inv);

	dgc_transform_t pose;
	dgc_transform_identity(pose);

	dgc_transform_t t;
	dgc_transform_read(t, filename.c_str());
	dgc_transform_left_multiply(t, to_kitti_t);
	dgc_transform_multiply(t, to_kitti_t_inv);
	dgc_transform_inverse(t, t);
	dgc_transform_multiply(pose, t);

	mpose = Eigen::Matrix4f::Identity();
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 4; col++) {
			mpose(row, col) = pose[row][col];
		}
	}
}

int main(int argc, char *argv[]) {
	if(argc != 5) {
		cerr << "ERROR, expected arguments: <correspondences-file> <cloud1.bin> <cloud2.bin> <transformation.tfm>" << endl;
		return 1;
	}

	ifstream corresp_file(argv[1]);
	if(!corresp_file.is_open()) {
		perror(argv[1]);
		return 1;
	}

	vector<DMatch> matches;
	RNG& rng(theRNG());
	int train, query;
	while(true) {
		corresp_file >> train >> query;
		if(corresp_file.eof()) {
			break;
		}
		if(rng.uniform(0.0, 1.0) < 0.01) {
			matches.push_back(DMatch(query, train, 1.0));
		}
	}

	Eigen::Matrix4f pose;
	load_transformation(argv[4], pose);

	VelodynePointCloud train_cloud, query_cloud;
	VelodynePointCloud::fromKitti(argv[2], train_cloud);
	VelodynePointCloud::fromKitti(argv[3], query_cloud);

	pcl::transformPointCloud(query_cloud, query_cloud, pose);

	Visualizer3D().setColor(0, 0, 255).addPointCloud(train_cloud)
			.setColor(0, 255, 0).addPointCloud(query_cloud)
			.addMatches(matches, train_cloud, query_cloud)
			.show();
}
