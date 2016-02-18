# but_velodyne_lib

Open source library for Velodyne 3D LIDAR point clouds processing provided by Robo@FIT group at Brno University of Technology.

Currently, the odometry estimation is main contribution of this repository. Available source code implements:

 * **Collar Line Segments** algorithm for fast odometry estimation
 * RANSAC based point cloud registration using local features from RGB image
 * Visual loop detection by VFH (experimental)

### Building information
Library requires following dependencies to be installed in the system:

 * Eigen3 library
 * OpenCV (version 2.4.9 used)
 * and PCL 1.7 library.

As a simple CMake project, it can be built by executing:

```bash
cd but_velodyne_lib
mkdir bin; cd bin
cmake ..
make
```

### Collar Line Segments for Fast Odometry Estimation

Detailed **description** of algorithm can be found in publication (see *doc/ICRA16_submission.pdf*):

*Velas, M. Spanel, M. Herout, A.: Collar Line Segments for Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016*

For **demonstration**, script *scripts/demo.sh* has been prepared. It requires internet connection (for obtaining data samples) and successfully built library. Demo aligns 5 data frames of Velodyne LiDAR and displays resulting 3D map.

