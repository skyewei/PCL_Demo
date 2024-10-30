
#include <iostream>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/filters/radius_outlier_removal.h>
#include <pcl-1.10/pcl/filters/conditional_removal.h>

int process_remove_outliers(int argc, char **argv) {

    if (argc != 2) {
        std::cerr << "please specify command line arg '-r' or '-c' " << std::endl;
        exit(0);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto &point : cloud->points) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);       
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    if (strcmp(argv[1], "-r") == 0) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius(2);
        outrem.setKeepOrganized(true);
        outrem.filter(*cloud_filtered);
    }
    else if (strcmp(argv[1], "-c") == 0) {
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(true);
        condrem.filter(*cloud_filtered);
    }
    else {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }

    std::cerr << "cloud before filtering: " << std::endl;
    for (const auto &point: cloud->points)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    std::cerr << "cloud after filtering: " << std::endl;
    for (const auto &point: cloud_filtered->points)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return 0;
}