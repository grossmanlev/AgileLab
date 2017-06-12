// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////
// librealsense tutorial #3 - Point cloud generation //
///////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <cstdio>
#include <thread>
#include <mutex>

// Also include GLFW to allow for graphical display
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include <iostream>
#include <fstream>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

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



double yaw, pitch, lastX, lastY; int ml;
bool capture = false;
int frame_cap = 0;
std::ofstream myfile;
std::ofstream check_file;

std::mutex mtx;
pcl::PointCloud<pcl::PointXYZ>::Ptr view_cloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform;
#define out_cloud cloud_transform



pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);

void display_pointcloud() {
    pcl::visualization::CloudViewer viewer ("PointCloud Viewer");
    std::this_thread::sleep_for(std::chrono::seconds(4));
    viewer.showCloud(view_cloud);
    while(!viewer.wasStopped ()) {
        if(mtx.try_lock()) {
            viewer.showCloud(view_cloud);
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT) ml = action == GLFW_PRESS;
}
static double clamp(double val, double lo, double hi) { return val < lo ? lo : val > hi ? hi : val; }
static void on_cursor_pos(GLFWwindow * win, double x, double y)
{
    if(ml)
    {
        yaw = clamp(yaw - (x - lastX), -120, 120);
        pitch = clamp(pitch + (y - lastY), -80, 80);
    }
    lastX = x;
    lastY = y;
}

int main() try
{
    std::thread PC_Viewer (display_pointcloud);


    check_file.open("check.txt");
    check_file << 0;
    check_file.close();

    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->start();

    // Open a GLFW window to display our output
    glfwInit();
    GLFWwindow * win = glfwCreateWindow(1280, 960, "Librealsense Stream", nullptr, nullptr);
    glfwSetCursorPosCallback(win, on_cursor_pos);
    glfwSetMouseButtonCallback(win, on_mouse_button);
    glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
    {
        auto dev = reinterpret_cast<rs::device *>(glfwGetWindowUserPointer(win));
        if (action != GLFW_RELEASE) switch (key)
        {
            case GLFW_KEY_C: {
                printf("Capturing...");
                capture = true;
            }
        }
    });
    glfwMakeContextCurrent(win);

    while(!glfwWindowShouldClose(win))
    {
        // Wait for new frame data
        glfwPollEvents();
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();

        // Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60, (float)1280/960, 0.01f, 20.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0,0,0, 0,0,1, 0,-1,0);
        glTranslatef(0,0,+0.5f);
        glRotated(pitch, 1, 0, 0);
        glRotated(yaw, 0, 1, 0);
        glTranslatef(0,0,-0.5f);

        // We will render our depth data as a set of points in 3D space
        glPointSize(2);
        glEnable(GL_DEPTH_TEST);
        glBegin(GL_POINTS);

        if(frame_cap % 6 == 0)
        {
            frame_cap = 0;
            capture = true;
        }
        frame_cap++;


        pcl::PointCloud<pcl::PointXYZ> cloud;
        int num_lines = depth_intrin.height*depth_intrin.width;
        // for(int dy=0; dy<depth_intrin.height; ++dy)
        // {
        //     for(int dx=0; dx<depth_intrin.width; ++dx)
        //     {
        //         uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
        //         if(depth_value == 0)
        //             num_lines--;
        //     }
        // }
        cloud.width    = num_lines;
        cloud.height   = 1;
        cloud.is_dense = true; //changed from "false"
        cloud.points.resize (cloud.width * cloud.height);

        if(capture) {
            //myfile.open("tmp.txt");
            // Fill in the cloud data
        }
        int counter = 0;
        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) continue;

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
                {
                    glColor3ub(255, 255, 255);
                }
                else
                {
                    glColor3ubv(color_image + (cy * color_intrin.width + cx) * 3);
                }

                // Emit a vertex at the 3D location of this depth pixel
                glVertex3f(depth_point.x, depth_point.y, depth_point.z);
                if(dx % 4 == 0 && dy % 4 == 0)
                {
                    cloud.points[counter].x = depth_point.x;
                    cloud.points[counter].y = depth_point.y;
                    cloud.points[counter].z = depth_point.z;
                    counter++;
                }
                /*
                if(dy%1 == 0 && dx%1 == 0 && capture)
                {
                    myfile << (double)depth_point.x << " " << (double)depth_point.y << " " << (double)depth_point.z << "\n";
                } 
                */
            }
        }
        cloud.width = counter;
        cloud.points.resize (cloud.width * cloud.height);

        if(capture)
        {
            /* Grab the file-mutex */
            
            // check_file.open("check.txt");
            // check_file << 0;
            // check_file.close();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = cloud.makeShared();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = filter_pointcloud(cloud1);
            if(mtx.try_lock()) {
               view_cloud = cloud_filtered;
               mtx.unlock();
            }


            // int ret = pcl::io::savePCDFileASCII<pcl::PointXYZ> ("new_image.pcd", *cloud1);
            // if(ret != 0)
            //     printf("RETURN: %d\n", ret);
            // std::cerr << "Saved " << cloud.points.size () << " data points to image.pcd." << std::endl;
            // rename("new_image.pcd", "image.pcd");

            // /* Release the file-mutex */
            
            // check_file.open("check.txt");
            // check_file << 1;
            // check_file.close();

        }

        capture = false;
        glEnd();

        glfwSwapBuffers(win);
    }
    PC_Viewer.join();
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    
    // VoxelGrid Filter
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud1);
    vox.setLeafSize (0.01f, 0.01f, 0.01f);
    vox.filter (*cloud2);

    if(false) {
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
          return NULL;
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

    if(false) { //Clustering using RegionGrowing algorithm (clusters based on "smoothness")
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
        /*
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
        */
    }
    return cloud2;
}

