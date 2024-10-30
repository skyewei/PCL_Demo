
#include "pass.h"

#include <iostream>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/filters/passthrough.h>

int processPassthroughFilter() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point: *cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
    
    // cereate a passthrough filter and set the parameters
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1);
    pass.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return 0;
}
