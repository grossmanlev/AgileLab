cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(voxel_grid)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (voxel_filter voxel_filter.cpp)
target_link_libraries (voxel_filter ${PCL_LIBRARIES})

add_executable (image_convert image_convert.cpp)
target_link_libraries (image_convert ${PCL_LIBRARIES})

add_executable (octree_viewer octree_viewer.cpp)
target_link_libraries (octree_viewer ${PCL_LIBRARIES})

add_executable (mesh_sampling mesh_sampling.cpp)
target_link_libraries (mesh_sampling ${PCL_LIBRARIES})