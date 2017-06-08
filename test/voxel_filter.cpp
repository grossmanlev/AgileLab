#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#define MIN  0

int main (int argc, char** argv)
{
  bool only_voxelize = true;

  if(argc < 3) {
    fprintf(stderr, "Example: ./voxel_filter [in_name].pcd [out_name].txt (bool_only_voxelize)\n");
    return 1;
  }
  if(argc == 4) {
    only_voxelize = strcmp(argv[3], "false");
    printf("only_voxelize %i", only_voxelize);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZ>);
  #define out_cloud cloud_transform
  
  // Fill in the cloud data
  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud1);
  std::cerr << "Saved" << cloud1->points.size() << " data points" << std::endl;

  // VoxelGrid Filter
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (cloud1);
  vox.setLeafSize (0.01f, 0.01f, 0.01f);
  vox.filter (*cloud2);
  cloud1->empty();

  if(!only_voxelize) {
    printf("SAC Segmentation\n");
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
    Eigen::Vector3f axis; axis << 0.0, 1.0, 0.0;
    seg.setAxis(axis);
    //seg.setEpsAngle(0.5);

    seg.setInputCloud (cloud2);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    Eigen::Vector3f n1(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f n2(0.0, 0.0, 1.0);
    float angle_rad = acos(n1.dot(n2) / (n1.norm() * n2.norm()));
    printf("Radians: %f\n", angle_rad);
    if(angle_rad < 0.3) { // planes is less than 17 degrees from z = 0
      printf("Taking out plane!\n");
      inliers->indices.clear();
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud2);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud1);
    cloud2->empty();
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
    
    // Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud1);
    sor.setMeanK(25);
    sor.setStddevMulThresh(1);
    sor.filter(*cloud2);
    cloud1->empty();
  }
  
  // Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(cloud2);
  sor2.setMeanK(25);
  sor2.setStddevMulThresh(2);
  sor2.filter(*cloud1);
  cloud2->empty();


  if(true) { //Clustering using RegionGrowing algorithm (clusters based on "smoothness")
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud1);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud1);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ()) {}
    

    //Saving individual clusters into .pcd files
    pcl::PCDWriter writer;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud1->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
    }
  }


  Eigen::Matrix<float,4,1> m;
  pcl::compute3DCentroid(*cloud1, m);
  printf("%f %f %f %f\n", m(0,0), m(1,0), m(2,0), m(3,0));



  
  Eigen::Vector4f min_point, max_point;
  pcl::getMinMax3D(*cloud1, min_point, max_point); //get the min/max point in the PC
  printf("Min Point: %f %f %f\n", min_point[0], min_point[1], min_point[2]);
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity(); //transformation
  //transform.translation() << -min_point[0], -min_point[1], -min_point[2]; //define the translation
  transform.translation() << 0.15-m(0,0), 0.15-m(1,0), 0.15-m(2,0);
  pcl::transformPointCloud(*cloud1, *cloud_transform, transform); //translate the PC

  printf("Width: %d, Height: %d\n", out_cloud->width, out_cloud->height);
  Eigen::Vector3i divs = vox.getNrDivisions();
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

  outfile.close();
  
  outfile.open("transformation.txt");
  outfile << " " << minX << " " << minY << " " << minZ << std::endl;
  outfile.close();

  printf("Min (x,y,z): (%d,%d,%d)\n", minX, minY, minZ);

  v = vox.getGridCoordinates(min_point[0], min_point[1], min_point[2]);
  printf("Min (x,y,z): (%d,%d,%d)\n", v[0], v[1], v[2]);

  pcl::io::savePCDFileASCII<pcl::PointXYZ> ("filtered.pcd", *out_cloud);


  return 0;
}