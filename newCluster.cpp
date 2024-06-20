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

    // "::Ptr" here is Type Alias
    // Ptr is a type of shared pointers, a smart pointer, it can do Garbage Collection (GC) automatically
    MyPointCloud::Ptr cloud(new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered(new MyPointCloud);

    // give the folder path of input and the input data name, it should be in the format of pcd
    std::string dir = R"(D:\GithubRepository\3D_DBSCAN4Virus\)";
    std::string filename = "testdata.pcd";

    // pcl is namespace, io is sub-namespace, loadPCDFile is function inside
    // *cloud for passing parameters，& cloud for receive "Pointers passed by reference"
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

    // The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
    // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
    // and also indexes all non-finite points of cloud_in
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 100000.0);
    // pass.setNegative(true); // keep reversed points
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


        pcl::visualization::PCLVisualizer viewer("Cloud viewer"); // create a visualized window
        viewer.addCoordinateSystem(1, std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center));
        viewer.setBackgroundColor(0.1, 0.1, 0.1);
        // pos: the position of camera, view: center of view, up: view direction
        viewer.setCameraPosition(std::get<0>(cloud_center), std::get<1>(cloud_center), 10, std::get<0>(cloud_center),
                                 std::get<1>(cloud_center), 30, 0, 0, 0);

        // create a color processing object: PointCloudColorHandlerRGB
        // PCLVisualizer Class use such object to display custom color data
        // In this case, PointCloudColorHandlerRGB object can get the RGB value of each point
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_filtered);
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

    return 0;



//------------------------ time test---------------//
//    // create a PointCloud object
//    MyPointCloud::Ptr cloud(new MyPointCloud);
//
//    // generate 100,000 point randomly
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
//    // get current time
//    auto end = std::chrono::high_resolution_clock::now();
//
//    // calculate time difference
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//
//    // Output the running time (microsecond)
//    std::cout << "Time taken by function: " << duration.count() / 1000 << " microseconds" << std::endl;
//
//    return 0;
}
