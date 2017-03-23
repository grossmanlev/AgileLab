#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/PCLPointCloud2.h>

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
  //pcl::PointCloud::Ptr cloud (new pcl::PointCloud ());
  //pcl::PointCloud::Ptr cloud_filtered (new pcl::PointCloud ());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data

  
  // pcl::PCDReader reader;
  // // Replace the path below with the path where you saved your file
  // reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud);

  //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
  //     << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  /*
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  */

  /*
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  */

  pcl::io::savePCDFileASCII<pcl::PointXYZ> ("filtered.pcd", *cloud_filtered);


  return 0;
}