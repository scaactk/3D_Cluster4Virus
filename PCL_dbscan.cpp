//
// Created by scaactk on 8/6/2023.
//
#include "dbscan.hpp"
#include "give_color.hpp"

int main(int argc, char **argv) {
    std::cout << "Begin reading PCL data" << std::endl;

    // "::Ptr" here is Type Alias 别名
    // Ptr是共享指针类型, 智能指针, 自动gc
    MyPointCloud::Ptr cloud(new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered(new MyPointCloud);

    std::string dir = R"(C:\Users\scaactk\Desktop\test\1\)";
    std::string filename = "1.pcd";

    // pcl is namespace, io is sub-namespace, loadPCDFile is function inside
    // *cloud 传参，& cloud接收 "通过引用传递指针"
    if (pcl::io::loadPCDFile<PointT>((dir + filename), *cloud) == -1) {
        // load file
        PCL_ERROR ("Couldn't read PCD file \n");
        return (-1);
    }
    cout << "there are " << cloud->points.size()<<" points before filtering." << endl;

//    pcl::PointXYZRGB minPt, maxPt;
//    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // filter
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);

    // problems here x is not filtered
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 10000.0);
    // pass.setNegative(true); // keep reversed points
    pass.filter(* cloud_filtered);
    //std::cout <<"aaaa";

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 10000.0);
    // pass.setNegative(true);
    pass.filter(* cloud_filtered);

    std::cout << "Filtered point cloud size: " << cloud_filtered->size() << " data points." << std::endl;


    // use ::max() to use static function, get min/max value in cpp definition
    // set initial value
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    // size_t 无符号整数， 在大多数平台上，std::size_t 的范围上限大约是 2^32 - 1 或 2^64 - 1，即为不会溢出，相比于 int i=0
    for (size_t i=0; i<cloud_filtered->size(); i++){
        // 箭头左边必须是指针；点左边必须是实体
        cloud_filtered->points[i].clusterID = 0;

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


    printf("Loaded %d data points from PCD\n", cloud_filtered->width * cloud_filtered->height);

    float mean_x = sum_x / cloud_filtered->size();
    float mean_y = sum_y / cloud_filtered->size();
    float mean_z = sum_z / cloud_filtered->size();
    std::cout << "X coordinate - Min: " << min_x << ", Max: " << max_x << ", Mean: " << mean_x << std::endl;
    std::cout << "Y coordinate - Min: " << min_y << ", Max: " << max_y << ", Mean: " << mean_y << std::endl;
    std::cout << "Z coordinate - Min: " << min_z << ", Max: " << max_z << ", Mean: " << mean_z << std::endl;

    size_t clusterNumber = dbscan(*cloud_filtered, 100.0, 3);
    std::cout<< "cluster size is "<< clusterNumber << std::endl;

    size_t coloredNumber = give_color(*cloud_filtered, clusterNumber);
    for (size_t i=0; i<cloud_filtered->size(); i++){
        if(cloud_filtered->points[i].clusterID==1){
            cout<<"color: "<< cloud_filtered->points[i].rgb<<endl;
        }
    }



    pcl::visualization::PCLVisualizer viewer("Cloud viewer"); //创建窗口
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    // pos: the position of camera, view: center of view, up: view direction
    viewer.setCameraPosition(mean_x, mean_y, 100000,mean_x, mean_y , 1000, 0, 0 , 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_filtered); //创建一个颜色处理对象PointCloudColorHandlerRGB，PCLVisualizer类利用这样的对象显示自定义颜色数据，在这个示例中，PointCloudColorHandlerRGB对象得到每个点云的RGB颜色字段
    viewer.addPointCloud<PointT> (cloud_filtered, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");


    while (!viewer.wasStopped())
        viewer.spinOnce(100);
    std::cout<<"sb"<<std::endl;
    std::cout<<"sb"<<std::endl;
    std::cout<<"sb"<<std::endl;
    std::cout<<"sb"<<std::endl;
    return 0;
}
