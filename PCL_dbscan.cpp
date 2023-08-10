//
// Created by scaactk on 8/6/2023.
//
# include <iostream>
# include <pcl/common/common_headers.h>
# include <pcl/io/pcd_io.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/visualization/cloud_viewer.h>
# include <pcl/console/parse.h>

int main(int argc, char **argv) {
    std::cout << "Begin reading PCL data" << std::endl;
    typedef pcl::PointXYZRGB PointT;

    // "::Ptr" here is Type Alias 别名
    // Ptr是共享指针类型
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointT>);

    std::string dir = R"(C:\Users\scaactk\Desktop\test\1\)";
    std::string filename = "1.pcd";

    // pcl is namespace, io is sub-namespace, loadPCDFile is function inside
    // *cloud 传参，& cloud接收 "通过引用传递指针"
    if (pcl::io::loadPCDFile<PointT>((dir + filename), *cloud) == -1) {
        // load file
        PCL_ERROR ("Couldn't read PCD file \n");
        return (-1);
    }
    printf("Loaded %d data points from PCD\n", cloud->width * cloud->height);

    // use ::max() to use static function, get min/max value in cpp definition
    // set initial value
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::min();

    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    // size_t 无符号整数， 在大多数平台上，std::size_t 的范围上限大约是 2^32 - 1 或 2^64 - 1，即为不会溢出，相比于 int i=0
    for (std::size_t i = 0; i < cloud->size(); i++) {
        // 箭头左边必须是指针；点左边必须是实体
        min_x = std::min(min_x, cloud->points[i].x);
        min_y = std::min(min_x, cloud->points[i].y);
        min_z = std::min(min_z, cloud->points[i].z);
        max_x = std::max(max_x, cloud->points[i].x);
        max_y = std::max(max_x, cloud->points[i].y);
        max_z = std::max(max_z, cloud->points[i].z);

        sum_x += cloud->points[i].x;
        sum_y += cloud->points[i].y;
        sum_z += cloud->points[i].z;
    }

    float mean_x = sum_x / cloud->size();
    float mean_y = sum_y / cloud->size();
    float mean_z = sum_z / cloud->size();
    std::cout << "X coordinate - Min: " << min_x << ", Max: " << max_x << ", Mean: " << mean_x << std::endl;
    std::cout << "Y coordinate - Min: " << min_y << ", Max: " << max_y << ", Mean: " << mean_y << std::endl;
    std::cout << "Z coordinate - Min: " << min_z << ", Max: " << max_z << ", Mean: " << mean_z << std::endl;



    pcl::visualization::PCLVisualizer viewer("Cloud viewer");

    // pos: the position of camera, view: center of view, up: view direction
    viewer.setCameraPosition(25000, 25000, 100000,25000, 25000 , 1000, 0, 0 , 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


    while (!viewer.wasStopped())
        viewer.spinOnce(100);

    return 0;
}