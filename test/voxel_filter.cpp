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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#define MIN  0

int main (int argc, char** argv)
{
  //pcl::PointCloud::Ptr cloud (new pcl::PointCloud ());
  //pcl::PointCloud::Ptr cloud_filtered (new pcl::PointCloud ());


  if(argc != 3) {
    fprintf(stderr, "Example: ./voxel_filter [in_name].pcd [out_name].txt\n");
    return 1;
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered4 (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  #define out_cloud cloud_filtered4
  
  // pcl::PCDReader reader;
  // // Replace the path below with the path where you saved your file
  // reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);

  //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
  //     << " data points (" << pcl::getFieldsList (*cloud) << ").";

  
  std::cerr << "Saved" << cloud->points.size() << " data points" << std::endl;



  // SAC Segmentation to Find Planes

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered1);


  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered1);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.5);
  sor.filter(*cloud_filtered2);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(cloud_filtered2);
  sor2.setMeanK(50);
  sor2.setStddevMulThresh(0.5);
  sor2.filter(*cloud_filtered3);


  // Create the filtering object

  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (cloud_filtered3);
  vox.setLeafSize (0.01f, 0.01f, 0.01f);
  vox.filter (*cloud_filtered4);

  /*
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  */

  /*
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  */
  printf("Width: %d, Height: %d\n", out_cloud->width, out_cloud->height);
  Eigen::Vector3i v3 = vox.getGridCoordinates(out_cloud->points[0].x,
    out_cloud->points[0].y,
    out_cloud->points[0].z);
  Eigen::Vector3i divs = vox.getNrDivisions();

  printf("[I,J,K]: [%d,%d,%d]\n", v3[0], v3[1], v3[2]);
  printf("%d %d %d\n", divs[0], divs[1], divs[2]);

  std::ofstream outfile;
  outfile.open(argv[2]);

  Eigen::Vector3i v;
  int minX, minY, minZ;
  minX = minY = minZ = MIN;

  for(int i = 0; i < out_cloud->points.size(); i++) {
    v = vox.getGridCoordinates(out_cloud->points[i].x,
      out_cloud->points[i].y,
      out_cloud->points[i].z);
    if(v[0] < minX)
      minX = v[0];
    if(v[1] < minY)
      minY = v[1];
    if(v[2] < minZ)
      minZ = v[2];
    outfile << " " << v[0] << " " << v[1] << " " << v[2] << std::endl;
  }

  //TODO: transform the points to be aligned properly along (0,0,0) -- this being absolute minimum point, so entire object is in the 1st quadrant?

  outfile.close();
  
  outfile.open("transformation.txt");
  outfile << " " << minX << " " << minY << " " << minZ << std::endl;
  outfile.close();

  printf("Min (x,y,z): (%d,%d,%d)\n", minX, minY, minZ);

  /*
  //Testing
  cloud_filtered->width++;
  cloud_filtered->points.resize(cloud_filtered->width*cloud_filtered->height);
  cloud_filtered->points[cloud_filtered->points.size()-1].x = 1;
  cloud_filtered->points[cloud_filtered->points.size()-1].y = 1;
  cloud_filtered->points[cloud_filtered->points.size()-1].z = 1;
  */

  pcl::io::savePCDFileASCII<pcl::PointXYZ> ("filtered.pcd", *out_cloud);


  return 0;
}