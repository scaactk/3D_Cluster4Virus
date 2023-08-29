//
// Created by scaactk on 8/14/2023.
//

#ifndef PCL_TEST_GIVE_COLOR_HPP
#define PCL_TEST_GIVE_COLOR_HPP

#include "PCL_TEST_HEADER.h"
#include <random>
#endif //PCL_TEST_GIVE_COLOR_HPP

void generate_color(std::array<uint8_t , 3> &color){
    std::random_device rd;
    std::mt19937 gen(rd()); // random engine
    std::uniform_int_distribution<int> dis(50,256); //define an uniform distribution
    color[0] = dis(gen);
    color[1] = dis(gen);
    color[2] = dis(gen);
}

bool single_color(MyPointCloud &cloud, const int queryID){
    std::random_device rd;
    std::mt19937 gen(rd()); // random engine
    std::uniform_int_distribution<int> dis(50,256); //define an uniform distribution
    uint8_t R = dis(gen);
    uint8_t G = dis(gen);
    uint8_t B = dis(gen);
    bool flag = false;
    for (int i=0; i < cloud.size(); i++){
        if (cloud.points[i].clusterID == queryID){
            flag = true;
            cloud.points[i].r = R;
            cloud.points[i].g = G;
            cloud.points[i].b = B;
//            cloud.points[i].r = 255;
//            cloud.points[i].g = 255;
//            cloud.points[i].b = 255;
        }
    }
    if (flag==false){
        return false;
    }
    return true;
}

void set_gray(MyPointCloud &cloud){
    for (int i=0; i<cloud.size(); i++){
        if(cloud.points[i].clusterID<0){ //-1 as noise point
            cloud.points[i].r = 50;
            cloud.points[i].g = 50;
            cloud.points[i].b = 50;
            cloud.points[i].a = 50;
        }
    }
}

int give_color(MyPointCloud &cloud, int clusterNumber){
    int count = 0;
    for (int i=0; i<cloud.size(); i++){
        if(single_color(cloud, i+1)){
            count++;
        }
    }
    set_gray(cloud);
    return count;
}
