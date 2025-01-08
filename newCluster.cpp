//
// Created by scaactk on 8/6/2023.
//
#include <pcl/point_cloud.h>

#include "include/PCL_TEST_HEADER.h"
#include "include/statistics.hpp"
#include "include/optics_new.hpp"
#include "include/give_color.hpp"
#include <filesystem>

#include <fstream>
#include "include/cluster_processor.hpp"

int main(int argc, char** argv)
{
    std::cout << "Begin reading PCL data" << std::endl;

    // "::Ptr" here is Type Alias
    // Ptr is a type of shared pointers, a smart pointer, it can do Garbage Collection (GC) automatically
    MyPointCloud::Ptr cloud(new MyPointCloud);
    MyPointCloud::Ptr cloud_filtered(new MyPointCloud);

    // give the folder path of input and the input data name, it should be in the format of pcd
    std::string dir = R"(C:/Users/tjut_/Desktop/3D_Cluster4Virus)";
    std::string filename = "Red.pcd";
    std::string new_path = dir + "/" + filename;
    // std:: cout << "aaa" << new_path << std::endl;

    // pcl is namespace, io is sub-namespace, loadPCDFile is function inside
    // *cloud for passing parameters，& cloud for receive "Pointers passed by reference"
    if (pcl::io::loadPCDFile<PointT>(new_path, *cloud) == -1)
    {
        // load file
        PCL_ERROR("Couldn't read PCD file \n");
        return (-1);
    }
    cout << "there are " << cloud->points.size() << " points before filtering." << endl;

    // 3D to 2D for test
    // for (int i = 0; i < cloud->points.size(); i++) {
    //     cloud->points[i].z = 0;
    // }

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

    // give pointID to filtered points
    for (int i = 0; i < cloud_filtered->points.size(); i++) {
        cloud_filtered->points[i].pointID = i;
    }

    std::tuple<float, float, float> cloud_center = statistics(*cloud_filtered);

    std::cout << "Start building kdtree" << std::endl;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_filtered);
    std::cout << "Finish building kdtree" << std::endl;

    // extract the short name of input file
    std::string filename_no_ext = filename.substr(0, filename.find_last_of('.'));
    std::cout<< "filename_no_ext: " << filename_no_ext<<std::endl;

    // To storage ordered_id & reachable_distance
    std::tuple<std::vector<int>, std::vector<float>> order_result;

    if (std::filesystem::exists(dir + "/" + filename_no_ext + "_idx_point.csv")) {
        std::cout << "OPTICS cache file already exists" << std::endl;
        std::ifstream file(dir + "/" + filename_no_ext + "_idx_point.csv");
        if (!file.is_open()) {
            std::cerr << "Error opening OPTICS cache file" << std::endl;
        }

        // reconstruct a new empty cloud
        MyPointCloud::Ptr new_cloud(new MyPointCloud);
        new_cloud->points.clear();

        // read data line by line
        std::string line;
        std::vector<int> ordered_sequence;
        std::vector<float> output_dist;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            std::vector<float> values;

            while (std::getline(iss, token, ',')) { //将iss中读取的存入token中，按','分割
                values.emplace_back(std::stof(token)); // std::stof, string to float
            }

            if (values.size() == 5) {
                PointT new_point;
                ordered_sequence.emplace_back(values[0]);
                new_point.pointID = values[0];
                output_dist.emplace_back(values[1]);
                new_point.x = values[2];
                new_point.y = values[3];
                new_point.z = values[4];

                new_cloud->points.push_back(new_point);
            }

        }
        new_cloud->width = new_cloud->points.size();
        new_cloud->height = 1;
        new_cloud->is_dense = true;
        std::cout << "New cloud size: " << new_cloud->points.size() << std::endl;

        order_result = std::make_tuple(ordered_sequence, output_dist);
    }
    else {
        std::cout << "Start OPTICS algorithm." << std::endl;
        order_result = optics_new(*cloud_filtered, kdtree, 1000, 4, dir, filename_no_ext);
    }

    float filter = 0;
    std::cout << "input filter" << std::endl;
    while (std::cin >> filter)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("Alpha Shape Viewer"));
        int clusterNumber = ClusterProcessor::process_clusters(*cloud_filtered,
                                                               dir,
                                                               filename_no_ext,
                                                               cloud_center,
                                                               std::get<0>(order_result),
                                                               std::get<1>(order_result),
                                                               filter,
                                                               viewer,
                                                               4);
        std::cout << "Cluster Number is " << clusterNumber << std::endl;

        // std::cout << "Start giving color" << std::endl;
        // set_gray(*cloud_filtered);
        // int coloredNumber = give_color(*cloud_filtered, clusterNumber);
        // std::cout << "Finish giving color" << std::endl;


        // pcl::visualization::PCLVisualizer viewer("Cloud viewer"); // create a visualized window
        // viewer.addCoordinateSystem(1, std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center));
        // viewer.setBackgroundColor(0.1, 0.1, 0.1);
        // // pos: the position of camera, view: center of view, up: view direction
        // viewer.setCameraPosition(std::get<0>(cloud_center), std::get<1>(cloud_center), 10, std::get<0>(cloud_center),
        //                          std::get<1>(cloud_center), 30, 0, 0, 0);
        //
        // // create a color processing object: PointCloudColorHandlerRGB
        // // PCLVisualizer Class use such object to display custom color data
        // // In this case, PointCloudColorHandlerRGB object can get the RGB value of each point
        // pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_filtered);
        // viewer.addPointCloud<PointT>(cloud_filtered, "sample cloud");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

        std::cout << "input new filter value" << endl;
    }

    return 0;
}
