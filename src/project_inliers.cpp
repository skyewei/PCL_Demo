
#include <iostream>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/ModelCoefficients.h>
#include <pcl-1.10/pcl/filters/project_inliers.h>

int process_project_inliers() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto &point : cloud->points) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0);
        point.y = 1024 * rand() / (RAND_MAX + 1.0);
        point.z = 1024 * rand() / (RAND_MAX + 1.0);
    }

    std::cerr << "Cloud before projection: " << std::endl;
    for (const auto &point : cloud->points) 
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0.0;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (const auto &point : cloud_projected->points) 
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
    
    return 0;
}