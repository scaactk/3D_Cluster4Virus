//
// Created by scaactk on 8/11/2023.
// definition of data structure
//

#ifndef PCL_TEST_HEADER_H
#define PCL_TEST_HEADER_H

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <chrono>
//# include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

// inherit PointXYZRGB and add new feature // not a good choose, it should use PCL’s build-in methods
//struct MyPoint : public pcl::PointXYZRGB {
//    //int label = 0; // center point / boundary point / noise point
//    int clusterID = 0;
//};
//typedef MyPoint PointT;
//typedef pcl::PointCloud<MyPoint> MyPointCloud;

// In modern C++ standard
// it does not require to explicit use 'typedef' when create a structure
struct EIGEN_ALIGN16 MyPoint{ //EIGEN_ALIGN16，Macro, align by 16 bit
    PCL_ADD_POINT4D; // default usage, Macro, define x,y,z,padding automatically
    PCL_ADD_RGB;
    int clusterID=0; // in old version, cannot be assigned directly
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //when use EIGEN_MAKE_ALIGNED_OPERATOR_NEW macro, do not need to add ';' in the end
    // Macro is the replacement of a piece of code, it has its ';' in the end
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint, // register point type Macro: XYZ + "clusterID" (as fields)
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, rgb, rgb)
                                  (int, clusterID, clusterID)
)
typedef MyPoint PointT;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

# endif //PCL_TEST_HEADER_H


