//
// Created by tjut_ on 8/25/2023.
//
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>

// 假设你已经包含了PCL库的头文件

int main() {
    std::string folderPath = "your_folder_path"; // 文件夹路径
    std::vector<std::string> fileNames; // 存储符合条件的文件名

    // 遍历文件夹，获取以3dlp结尾的文件名
    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (entry.path().extension() == ".3dlp") {
            fileNames.push_back(entry.path().string());
        }
    }

    // 逐个处理文件
    for (const auto& fileName : fileNames) {
        std::ifstream file(fileName);
        if (!file) {
            std::cerr << "Failed to open file: " << fileName << std::endl;
            continue;
        }

        // 创建PointCloud对象
        pcl::PointCloud<pcl::PointXYZ> cloud;
        std::string line;

        // 逐行读取文件内容
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            std::vector<double> values;

            // 使用tab符号分隔每列数据
            while (std::getline(iss, token, '\t')) {
                // 将前三列数据存储到values向量中
                if (values.size() < 3) {
                    values.push_back(std::stod(token));
                }
            }

            // 创建PointXYZ对象并添加到PointCloud中
            if (values.size() >= 3) {
                pcl::PointXYZ point;
                point.x = values[0];
                point.y = values[1];
                point.z = values[2];
                cloud.push_back(point);
            }
        }

        // 将PointCloud保存为PCD文件
        pcl::io::savePCDFileASCII("output.pcd", cloud);
        std::cout << "PCD file saved: output.pcd" << std::endl;
    }

    return 0;
}
