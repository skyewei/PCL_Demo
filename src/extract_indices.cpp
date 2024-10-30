
#include <iostream>
#include <pcl-1.10/pcl/ModelCoefficients.h>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/sample_consensus/method_types.h>
#include <pcl-1.10/pcl/sample_consensus/model_types.h>
#include <pcl-1.10/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.10/pcl/filters/voxel_grid.h>
#include <pcl-1.10/pcl/filters/extract_indices.h>

int process_extract_indices () {
   
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
        cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read ("../data/bun0.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);

    // convert to the templated point type
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size () << " data points." << std::endl;

    // write the downsampled data to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("../data/bun0-out_downsampled.pcd", *cloud_filtered, false);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points) {
        // segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // extract the iniers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->points.size () << " data points." << std::endl;

        std::stringstream ss;
        ss << "../data/bun0-table_scene_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }

    return 0;
}

