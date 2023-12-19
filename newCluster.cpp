//
// Created by scaactk on 8/6/2023.
//
#include "include/dbscan.hpp"
#include "include/dbscan_kdtree.hpp"
#include "include/optics.hpp"
#include "include/statistics.hpp"
#include "include/optics_new.hpp"

int main(int argc, char **argv) {
    std::cout << "Begin reading PCL data" << std::endl;

    // "::Ptr" here is Type Alias 别名
    // Ptr是共享指针类型, 智能指针, 自动gc
    MyPointCloud::Ptr cloud(new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered(new MyPointCloud);

    std::string dir = R"(C:\Users\scaactk\Desktop\)";
    std::string filename = "Red.pcd";

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
//    int clusterNumber = dbscan_kdtree(*cloud_filtered, kdtree, 100, 4);
    std::tuple<std::vector<int>, std::vector<float>> order_result = optics_new(*cloud_filtered, kdtree, 1000, 4, dir);
    float filter = 0;
    std::cout << "input filter" << endl;
    while(std::cin >> filter){
        int clusterNumber = extract_id_new(*cloud_filtered, std::get<0>(order_result), std::get<1>(order_result), filter);
        std::cout<< "Cluster NUmber is "<< clusterNumber << std::endl;

        std::cout << "Start giving color" << std::endl;
        set_gray(*cloud_filtered);
        int coloredNumber = give_color(*cloud_filtered, clusterNumber);
        std::cout << "Finish giving color" << std::endl;


        pcl::visualization::PCLVisualizer viewer("Cloud viewer"); //创建窗口
        viewer.addCoordinateSystem(1000, std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center));
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

        std::cout << "input new filter value" << endl;
    }


//    for (int i=0; i<cloud_filtered->size(); i++){
//        if(cloud_filtered->points[i].clusterID==1){
//            cout<<"color: "<< cloud_filtered->points[i].rgb<<endl;
//        }
//    }




    std::cout << "sb" << std::endl;
    std::cout << "sb" << std::endl;
    std::cout << "sb" << std::endl;
    std::cout << "sb" << std::endl;
    return 0;



//------------------------ time test---------------//
//    // 创建一个PointCloud对象
//    MyPointCloud::Ptr cloud(new MyPointCloud);
//
//    // 生成100,000个随机点
//    std::random_device rd;
//    std::mt19937 gen(rd()); // random engine
//    int m = 10000;
//    std::uniform_real_distribution<float> dis(0,10*m); //define an uniform distribution
//
//    const int num_points = 100*m;
//    for (int i = 0; i < num_points; ++i)
//    {
//        PointT point;
//        point.x = dis(gen);
//        point.y = dis(gen);
//        point.z = dis(gen);
//        cloud->push_back(point);
//    }
//
//
//    auto start = std::chrono::high_resolution_clock::now();
//    pcl::KdTreeFLANN<PointT> kdtree;
//    kdtree.setInputCloud(cloud);
//    dbscan_kdtree(*cloud,kdtree, 2 * m, 4);
//    // 获取当前时间点
//    auto end = std::chrono::high_resolution_clock::now();
//
//    // 计算时间差
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//
//    // 输出运行时间（微秒）
//    std::cout << "Time taken by function: " << duration.count() / 1000 << " microseconds" << std::endl;
//
//    return 0;
}
