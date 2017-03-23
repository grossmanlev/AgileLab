#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/filters/filter.h>


#include <boost/version.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>

int main (int argc, char** argv)
{
  if(argc != 2)
    return 1;

  std::string line;
  int lines;
  std::ifstream infile;
  infile.open(argv[1]);

  if(infile.is_open()) {
    for(lines = 0; std::getline(infile, line); lines++);
  }
  infile.close();

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = lines;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);


  float x, y, z;
  infile.open(argv[1]);
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    //std::getline(infile, line);
    //sscanf(line, "%f %f %f", &x, &y, &z);
    infile >> x >> y >> z;
    cloud.points[i].x = x;
    cloud.points[i].y = y;
    cloud.points[i].z = z;
  }
  infile.close();

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}