/*
 * Odometry estimation by collar line segments of Velodyne scan.
 *
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/04/2015
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
#include <libgen.h>

#include <pcl/common/eigen.h>

#include <cv.h>

#include <but_velodyne_odom/VelodynePointCloud.h>
#include <but_velodyne_odom/EigenUtils.h>
#include <but_velodyne_odom/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne_odom;

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

	if(argc == 1) {
		cerr << "Insufficient arguments. Usage: " << argv[0] << "<point-cloud>+" << endl;
		return EXIT_FAILURE;
	}

	char first_pose[256];
	strcpy(first_pose, argv[1]);
	string graph_filename = string(dirname(first_pose)) + "/poses.graph";
	ofstream graph_file(graph_filename.c_str());
	if (!graph_file.is_open()) {
		perror(graph_filename.c_str());
		exit(1);
	}

	CollarLinesRegistrationPipeline registration(
			CollarLinesRegistrationPipeline::linear_estimator, graph_file);

	VelodynePointCloud target_cloud;
	vector<Mat> covariances(argc - 1);
	for (int i = 1; i < argc; i++) {

		string filename = argv[i];
		log << "KITTI file: " << filename << endl << flush;
		if (filename.find(".pcd") != string::npos) {
			io::loadPCDFile(filename, target_cloud);
		} else {
			VelodynePointCloud::fromKitti(filename, target_cloud);
		}

		Eigen::Matrix4f t = registration.runRegistration(target_cloud,
				covariances[i - 1]);
		EigenUtils::saveMatrix(filename + string(".transform"), t);
		registration.output(t);
	}

	string cov_filename = string(dirname(argv[1])) + "/covariances.yaml";
	FileStorage cov_fs(cov_filename, FileStorage::WRITE);
	cov_fs << "covariances" << covariances;

	return EXIT_SUCCESS;
}
