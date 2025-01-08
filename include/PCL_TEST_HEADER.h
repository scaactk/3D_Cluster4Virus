#ifndef PCL_TEST_HEADER_H
#define PCL_TEST_HEADER_H

#include <pcl/point_cloud.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>

struct EIGEN_ALIGN16 MyPoint {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    int pointID;
    int clusterID =-1;

    MyPoint() : clusterID(0) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, rgb, rgb)
                                  (int, pointID, pointID)
                                  (int, clusterID, clusterID)
                                  )

using PointT = MyPoint;
using MyPointCloud = pcl::PointCloud<MyPoint>;

#endif
