//
// Created by scaactk on 8/11/2023.
//

#ifndef PCL_TEST_HEADER_H
#define PCL_TEST_HEADER_H

#define PCL_NO_PRECOMPILE //自定义点类型时，需要在包含任何PCL头文件之前定义PCL_NO_PRECOMPILE来包含模板化算法
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
//# include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

// inherit PointXYZRGB and add new feature 这种方式不好，很麻烦，该使用PCL自带的拓展点定义方式
//struct MyPoint : public pcl::PointXYZRGB {
//    //int label = 0; // center point / boundary point / noise point
//    int clusterID = 0;
//};
//typedef MyPoint PointT;
//typedef pcl::PointCloud<MyPoint> MyPointCloud;

// 现代的 C++ 标准不再要求在定义结构体时显式使用 typedef
struct EIGEN_ALIGN16 MyPoint{ //EIGEN_ALIGN16，宏，16字节对齐
    PCL_ADD_POINT4D; // 默认方式，宏，自动定义x,y,z,padding
    PCL_ADD_RGB;
    int clusterID=0; //旧版本似乎不能直接赋值
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //在使用 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 宏时，不需要在结尾添加分号; （混乱
    //宏是一个代码片段的替换，在其展开时会自动包含结尾的分号，为什么前面的宏后面有分号
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint, // 注册点类型宏 XYZ + "clusterID" (as fields)
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, rgb, rgb)
                                  (int, clusterID, clusterID)
)
typedef MyPoint PointT;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

# endif //PCL_TEST_HEADER_H


