//
// Created by scaactk on 8/6/2023.
//
# include "dbscan.hpp"

int main(int argc, char **argv) {
    std::cout << "Begin reading PCL data" << std::endl;

    // "::Ptr" here is Type Alias 别名
    // Ptr是共享指针类型
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string dir = R"(C:\Users\scaactk\Desktop\test\1\)";
    std::string filename = "1.pcd";

    // pcl is namespace, io is sub-namespace, loadPCDFile is function inside
    // *cloud 传参，& cloud接收 "通过引用传递指针"
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>((dir + filename), *cloud) == -1) {
        // load file
        PCL_ERROR ("Couldn't read PCD file \n");
        return (-1);
    }
    cout << "there are " << cloud->points.size()<<" points before filtering." << endl;

    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 5000.0);
    pass.filter(* cloud_filtered);
    std::cout <<"aaaa";

    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 5000.0);
    pass.filter(* cloud_filtered);

    std::cout << "Filtered point cloud size: " << cloud_filtered->size() << " data points." << std::endl;


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

    // convert PointRGBXYZ into MyPoint
    MyPointCloud::Ptr myCloud(new MyPointCloud);
    myCloud->resize(cloud_filtered->size());

    // size_t 无符号整数， 在大多数平台上，std::size_t 的范围上限大约是 2^32 - 1 或 2^64 - 1，即为不会溢出，相比于 int i=0
    for (size_t i=0; i<cloud_filtered->size(); i++){
        // 箭头左边必须是指针；点左边必须是实体
        myCloud->points[i].x = cloud_filtered->points[i].x;
        myCloud->points[i].y = cloud_filtered->points[i].y;
        myCloud->points[i].z = cloud_filtered->points[i].z;
        myCloud->points[i].r = cloud_filtered->points[i].r;
        myCloud->points[i].g = cloud_filtered->points[i].g;
        myCloud->points[i].b = cloud_filtered->points[i].b;
        myCloud->points[i].clusterID = 0;

        min_x = std::min(min_x, cloud_filtered->points[i].x);
        min_y = std::min(min_x, cloud_filtered->points[i].y);
        min_z = std::min(min_z, cloud_filtered->points[i].z);
        max_x = std::max(max_x, cloud_filtered->points[i].x);
        max_y = std::max(max_x, cloud_filtered->points[i].y);
        max_z = std::max(max_z, cloud_filtered->points[i].z);

        sum_x += cloud_filtered->points[i].x;
        sum_y += cloud_filtered->points[i].y;
        sum_z += cloud_filtered->points[i].z;
    }


    printf("Loaded %d data points from PCD\n", myCloud->width * myCloud->height);

    float mean_x = sum_x / myCloud->size();
    float mean_y = sum_y / myCloud->size();
    float mean_z = sum_z / myCloud->size();
    std::cout << "X coordinate - Min: " << min_x << ", Max: " << max_x << ", Mean: " << mean_x << std::endl;
    std::cout << "Y coordinate - Min: " << min_y << ", Max: " << max_y << ", Mean: " << mean_y << std::endl;
    std::cout << "Z coordinate - Min: " << min_z << ", Max: " << max_z << ", Mean: " << mean_z << std::endl;

    int num = dbscan(*myCloud, 20.0, 3);
    std::cout<< "cluster size is "<< num << std::endl;



//    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
//
//    // pos: the position of camera, view: center of view, up: view direction
//    viewer.setCameraPosition(25000, 25000, 100000,25000, 25000 , 1000, 0, 0 , 0);
//    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//
//
//    while (!viewer.wasStopped())
//        viewer.spinOnce(100);

    return 0;
}
