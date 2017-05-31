#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/PCLPointCloud2.h>

int main(int argc, char* argv[]) {
	if(argc != 2) {
		printf("Format: ./image_convert test_img.ply\n");
		return 1;
	}
	std::string filename = argv[1];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadOBJFile<pcl::PointXYZ> (filename, *cloud);
	pcl::io::savePCDFileASCII<pcl::PointXYZ> ("converted_img.pcd", *cloud);


	// pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());

	// pcl::PLYReader reader;
	// reader.read(filename, *cloud);

	// pcl::PCDWriter writer;
	// writer.write("converted_img.pcd", *cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	return 0;
}