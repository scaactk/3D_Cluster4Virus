//
// Created by scaactk on 8/11/2023.
//

# ifndef PCL_TEST_HEADER_H
# define PCL_TEST_HEADER_H

# include <iostream>
# include <pcl/common/common_headers.h>
# include <pcl/io/pcd_io.h>
# include <pcl/point_types.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/visualization/cloud_viewer.h>
# include <pcl/console/parse.h>
# include <pcl/filters/passthrough.h>
//# include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

// inherit PointXYZRGB and add new feature
struct MyPoint : public pcl::PointXYZRGB {
    //int label = 0; // center point / boundary point / noise point
    int clusterID = 0;
};
typedef MyPoint PointT;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

# endif //PCL_TEST_HEADER_H


