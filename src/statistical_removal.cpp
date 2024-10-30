
#include "statistical_removal.h"

#include <iostream>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>


void process_statistical_removal()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ> ("../data/bun0.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("../data/bun0-inlier.pcd", *cloud_filtered, false);

  sor.setNegative(true);
  sor.filter(*cloud_filtered);
  writer.write<pcl::PointXYZ> ("../data/bun0-outlier.pcd", *cloud_filtered, false);

}