cmake_minimum_required(VERSION 2.4.6)

project(but_velodyne_odom)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE RELEASE)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DBOOST_PARAMETER_MAX_ARITY=7)

include_directories( ${PROJECT_SOURCE_DIR}/include )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

###############################################################################

add_executable(collar-lines-odom src/apps/collar-lines-odom.cpp 
src/VelodynePointCloud.cpp src/PointCloudLine.cpp src/Visualizer3D.cpp src/LineCloud.cpp 
src/PolarGridOfClouds.cpp src/CollarLinesRegistration.cpp src/MoveEstimation.cpp
src/Termination.cpp src/CollarLinesRegistrationPipeline.cpp src/PoseGraphEdge.cpp)
target_link_libraries(collar-lines-odom ${PCL_LIBRARIES}) 
target_link_libraries(collar-lines-odom ${OpenCV_LIBS})

###############################################################################

add_executable(show-kitti-poses src/apps/show-kitti-poses.cpp src/VelodynePointCloud.cpp 
src/Visualizer3D.cpp)
target_link_libraries(show-kitti-poses ${PCL_LIBRARIES})
target_link_libraries(show-kitti-poses ${OpenCV_LIBS})

###############################################################################

add_executable(extract-vfh src/apps/extract-vfh.cpp src/VelodynePointCloud.cpp
src/Visualizer3D.cpp)
target_link_libraries(extract-vfh ${PCL_LIBRARIES})
target_link_libraries(extract-vfh ${OpenCV_LIBS})

add_executable(find-loops-vfh src/apps/find-loops-vfh.cpp src/VelodynePointCloud.cpp
src/Visualizer3D.cpp)
target_link_libraries(find-loops-vfh ${PCL_LIBRARIES})
target_link_libraries(find-loops-vfh ${OpenCV_LIBS})

add_executable(verify-loops src/apps/verify-loops.cpp src/Visualizer3D.cpp 
src/PointCloudLine.cpp src/PoseGraphEdge.cpp src/MoveEstimation.cpp)
target_link_libraries(verify-loops ${OpenCV_LIBS})
target_link_libraries(verify-loops ${PCL_LIBRARIES})

add_executable(register-loops src/apps/register-loops.cpp src/VelodynePointCloud.cpp
src/PointCloudLine.cpp src/Visualizer3D.cpp src/LineCloud.cpp src/PolarGridOfClouds.cpp
src/CollarLinesRegistration.cpp src/MoveEstimation.cpp src/Termination.cpp
src/CollarLinesRegistrationPipeline.cpp src/PoseGraphEdge.cpp)
target_link_libraries(register-loops ${PCL_LIBRARIES})
target_link_libraries(register-loops ${OpenCV_LIBS})

add_executable(slampp-solution-to-poses src/apps/slampp-solution-to-poses.cpp)
target_link_libraries(slampp-solution-to-poses ${OpenCV_LIBS})
target_link_libraries(slampp-solution-to-poses ${PCL_LIBRARIES})

###############################################################################

add_executable(sac-visual-registration src/apps/sac-visual-registration.cpp src/VelodynePointCloud.cpp
src/Visualizer3D.cpp src/KeypointsCorrespondenceProjector.cpp)
target_link_libraries(sac-visual-registration ${PCL_LIBRARIES})
target_link_libraries(sac-visual-registration ${OpenCV_LIBS})
