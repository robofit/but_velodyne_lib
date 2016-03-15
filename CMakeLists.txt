cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(but_velodyne_lib)
set (BUT_VELODYNE_VERSION_MAJOR 0)
set (BUT_VELODYNE_VERSION_MINOR 1)
set (BUT_VELODYNE_VERSION_PATCH 0)
set (BUT_VELODYNE_VERSION ${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR}.${BUT_VELODYNE_VERSION_PATCH})
set (BUT_VELODYNE_SOVERSION ${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR})
message(STATUS "Compiling But-Velodyne Version ${BUT_VELODYNE_VERSION}, soversion=${BUT_VELODYNE_SOVERSION}")

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")
set(BUT_VELODYNE_INCLUDE_INSTALL_DIR "but_velodyne-${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR}" CACHE STRING "Directory name for header files")
set(BUT_VELODYNE_CMAKE_INSTALL_DIR "but_velodyne-${BUT_VELODYNE_VERSION_MAJOR}.${BUT_VELODYNE_VERSION_MINOR}" CACHE STRING "Directory to install the cmake config files.")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_BUILD_TYPE RELEASE)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_definitions(-DBOOST_PARAMETER_MAX_ARITY=7)

include_directories( ${PROJECT_SOURCE_DIR}/include)

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

########################### Library: #########################################

file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/but_velodyne/*.h)
install(FILES ${header_files} DESTINATION include/${BUT_VELODYNE_INCLUDE_INSTALL_DIR}/but_velodyne COMPONENT ${COMPONENT_PREFIX}dev)
file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/velodyne_pointcloud/*.h)
install(FILES ${header_files} DESTINATION include/${BUT_VELODYNE_INCLUDE_INSTALL_DIR}/velodyne_pointcloud COMPONENT ${COMPONENT_PREFIX}dev)

add_library(but_velodyne SHARED src/CollarLinesRegistration.cpp src/CollarLinesRegistrationPipeline.cpp
src/ImageLine.cpp src/KeypointsCorrespondenceProjector.cpp src/LineCloud.cpp src/MoveEstimation.cpp
src/PointCloudLine.cpp src/PolarGridOfClouds.cpp src/PoseGraphEdge.cpp src/Termination.cpp
src/VelodynePointCloud.cpp src/Visualizer2DCorrespondences.cpp src/Visualizer3D.cpp)
target_include_directories(but_velodyne PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(but_velodyne ${PCL_LIBRARIES})
target_link_libraries(but_velodyne ${OpenCV_LIBS})
target_link_libraries(but_velodyne ${Boost_LIBRARIES})

########################### Odometry estimation: #############################

add_executable(collar-lines-odom src/apps/collar-lines-odom.cpp)
target_link_libraries(collar-lines-odom but_velodyne)

########################## Visualization apps: ###############################

add_executable(show-kitti-poses src/apps/show-kitti-poses.cpp)
target_link_libraries(show-kitti-poses but_velodyne)

add_executable(show_correspondences src/apps/show_correspondences.cpp)
target_link_libraries(show_correspondences but_velodyne)

########################## Loops detection/closure: ##########################

add_executable(extract-vfh src/apps/extract-vfh.cpp)
target_link_libraries(extract-vfh but_velodyne)

add_executable(find-loops-vfh src/apps/find-loops-vfh.cpp)
target_link_libraries(find-loops-vfh but_velodyne)

add_executable(verify-loops src/apps/verify-loops.cpp)
target_link_libraries(verify-loops but_velodyne)

add_executable(register-loops src/apps/register-loops.cpp)
target_link_libraries(register-loops but_velodyne)

add_executable(slampp-solution-to-poses src/apps/slampp-solution-to-poses.cpp)
target_link_libraries(slampp-solution-to-poses but_velodyne)

########################## RANSAC registration (visual features): ############

add_executable(sac-visual-registration src/apps/sac-visual-registration.cpp)
target_link_libraries(sac-visual-registration but_velodyne)

########################## Installation: #####################################

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/but_velodyne-config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config.cmake" IMMEDIATE @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/but_velodyne-config-version.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config-version.cmake" IMMEDIATE @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/but_velodyne-config-version.cmake"
    DESTINATION "lib${LIB_SUFFIX}/cmake/${BUT_VELODYNE_CMAKE_INSTALL_DIR}" COMPONENT ${COMPONENT_PREFIX}dev)

install(
TARGETS but_velodyne collar-lines-odom show-kitti-poses show_correspondences extract-vfh
    find-loops-vfh verify-loops register-loops slampp-solution-to-poses sac-visual-registration
RUNTIME DESTINATION bin/
LIBRARY DESTINATION lib/
INCLUDES DESTINATION include/
)
