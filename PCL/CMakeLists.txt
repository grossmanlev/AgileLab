cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL)
find_package(PCL 1.3 REQUIRED COMPONENTS common io octree visualization filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(octree_viewer octree_viewer.cpp)
target_link_libraries(octree_viewer ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_OCTREE_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES})