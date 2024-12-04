//
// Created by scaactk on 8/29/2023.
//

#ifndef PCL_TEST_STATISTICS_HPP
#define PCL_TEST_STATISTICS_HPP

#include "PCL_TEST_HEADER.h"
#include <pcl/point_cloud.h>

inline std::tuple<float, float, float> statistics(const MyPointCloud& cloud){

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

    for (int i = 0; i < cloud.size(); i++) {
        // 箭头左边必须是指针；点左边必须是实体

        min_x = std::min(min_x, cloud.points[i].x);
        min_y = std::min(min_x, cloud.points[i].y);
        min_z = std::min(min_z, cloud.points[i].z);
        max_x = std::max(max_x, cloud.points[i].x);
        max_y = std::max(max_x, cloud.points[i].y);
        max_z = std::max(max_z, cloud.points[i].z);

        sum_x += cloud.points[i].x;
        sum_y += cloud.points[i].y;
        sum_z += cloud.points[i].z;
    }


    printf("Loaded %d data points from PCD\n", cloud.width * cloud.height);

    float mean_x = sum_x / cloud.size();
    float mean_y = sum_y / cloud.size();
    float mean_z = sum_z / cloud.size();
    std::cout << "X coordinate - Min: " << min_x << ", Max: " << max_x << ", Mean: " << mean_x << std::endl;
    std::cout << "Y coordinate - Min: " << min_y << ", Max: " << max_y << ", Mean: " << mean_y << std::endl;
    std::cout << "Z coordinate - Min: " << min_z << ", Max: " << max_z << ", Mean: " << mean_z << std::endl;

    return std::make_tuple(mean_x, mean_y, mean_z);

}
#endif //PCL_TEST_STATISTICS_HPP
