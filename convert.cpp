//
// Created by tjut_ on 8/25/2023.
// this code is use for convert .3dlp data from SMLM to .pcd format that can be used in PointCloud
//
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include "include/PCL_TEST_HEADER.h"

namespace fs = boost::filesystem;

// path for input, custom define
std::string folderPath = "C:\\Users\\scaactk\\Desktop\\20231123";

void handle_file(const fs::path& path)
{
    // Step 1: First pass over the file to count lines (i.e., data points)
    std::ifstream infile_count_pass(path.string());
    std::string line;
    int point_count = 0;
    while (std::getline(infile_count_pass, line)){
        ++point_count;
    }
    infile_count_pass.close();

    // Step 2: Then, actually process data
    std::ifstream infile(path.string());
    std::ofstream outfile(path.string().substr(0, (path.string().length() - 5)) + ".pcd", ios::out);
    if(!outfile.is_open())
    {
        std::cerr << "Failed to open output file at: " << path.stem().string() + ".pcd" << std::endl;
        return;
    }
    std::cout << path.stem().string() + ".pcd" << endl;


    // Write the PCD file's header information
    outfile << "# .PCD v.7 - Point Cloud Data file format\n"
            << "VERSION .7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4\n"
            << "TYPE F F F\n"
            << "COUNT 1 1 1\n"
            << "WIDTH " << point_count << "\n"  // use the count from the first pass
            << "HEIGHT 1\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS " << point_count << "\n"  // use the count from the first pass
            << "DATA ascii\n";

    float x, y, z;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> x >> y >> z)
        {
            // Separate the data by tab and write the point cloud to new pcd file
            outfile << x << "\t" << y << "\t" << z << "\n";
           // std::cout << x <<endl;
        }
    }

    infile.close();
    outfile.close();

}

int main()
{
    fs::path p(folderPath);
    fs::recursive_directory_iterator begin(p), end;
    std::vector<fs::directory_entry> v(begin, end);

    //iterate through all files
    for (auto& f : v)
    {
        // check the extension of file
        if (f.path().extension() == ".3dlp")
        {
            std::cout<<f.path()<<std::endl;
            handle_file(f.path());
        }
    }

    return 0;
}
