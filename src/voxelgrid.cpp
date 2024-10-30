
#include "voxelgrid.h"

#include <iostream>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/filters/voxel_grid.h>

int processVoxelGrid() {

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);

    pcl::PCDReader reader;
    reader.read ("../data/bun0.pcd", *cloud);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * 
        cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) 
        << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write ("../data/bun0-voxelized.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), 
        Eigen::Quaternionf::Identity(), false);

    return 0;
}