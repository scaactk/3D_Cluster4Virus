//
// Created by scaactk on 8/6/2023.
//
#include "include/dbscan.hpp"
#include "include/dbscan_kdtree.hpp"
#include "include/optics.hpp"
#include "include/statistics.hpp"

int main(int argc, char **argv) {
    std::cout << "Begin reading PCL data" << std::endl;

    // "::Ptr" here is Type Alias 别名
    // Ptr是共享指针类型, 智能指针, 自动gc
    MyPointCloud::Ptr cloud(new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered(new MyPointCloud);

    std::string dir = R"(C:\Users\tjut_\Desktop\20230818_VLP samples\)";
    std::string filename = "1.pcd";

    // pcl is namespace, io is sub-namespace, loadPCDFile is function inside
    // *cloud 传参，& cloud接收 "通过引用传递指针"
    if (pcl::io::loadPCDFile<PointT>((dir + filename), *cloud) == -1) {
        // load file
        PCL_ERROR ("Couldn't read PCD file \n");
        return (-1);
    }
    cout << "there are " << cloud->points.size() << " points before filtering." << endl;

//    pcl::PointXYZRGB minPt, maxPt;
//    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // filter
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);

    // problems here x is not filtered
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 100000.0);
    // pass.setNegative(true); // keep reversed points
    pass.filter(*cloud_filtered);
    //std::cout <<"aaaa";

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 100000.0);
    // pass.setNegative(true);
    pass.filter(*cloud_filtered);

    std::cout << "Filtered point cloud size: " << cloud_filtered->size() << " data points." << std::endl;

    std::tuple<float, float, float> cloud_center = statistics(*cloud_filtered);

    std::cout << "Start building kdtree" << std::endl;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_filtered);
    std::cout << "Finish building kdtree" << std::endl;


//    int clusterNumber = dbscan(*cloud_filtered, 100.0, 4);
    //int clusterNumber = dbscan_kdtree(*cloud_filtered, kdtree, 30, 4);
    int clusterNumber = optics(*cloud_filtered, kdtree, 2000, 10);
    std::cout<< "Cluster NUmber is "<< clusterNumber << std::endl;

    std::cout << "Start giving color" << std::endl;
    set_gray(*cloud_filtered);
    int coloredNumber = give_color(*cloud_filtered, clusterNumber);
    std::cout << "Finish giving color" << std::endl;
//    for (int i=0; i<cloud_filtered->size(); i++){
//        if(cloud_filtered->points[i].clusterID==1){
//            cout<<"color: "<< cloud_filtered->points[i].rgb<<endl;
//        }
//    }



    pcl::visualization::PCLVisualizer viewer("Cloud viewer"); //创建窗口
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    // pos: the position of camera, view: center of view, up: view direction
    viewer.setCameraPosition(std::get<0>(cloud_center), std::get<1>(cloud_center), 10000, std::get<0>(cloud_center),
                             std::get<1>(cloud_center), 1000, 0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(
            cloud_filtered); //创建一个颜色处理对象PointCloudColorHandlerRGB，PCLVisualizer类利用这样的对象显示自定义颜色数据，在这个示例中，PointCloudColorHandlerRGB对象得到每个点云的RGB颜色字段
    viewer.addPointCloud<PointT>(cloud_filtered, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");


    while (!viewer.wasStopped())
        viewer.spinOnce(100);
    std::cout << "sb" << std::endl;
    std::cout << "sb" << std::endl;
    std::cout << "sb" << std::endl;
    std::cout << "sb" << std::endl;
    return 0;
}
