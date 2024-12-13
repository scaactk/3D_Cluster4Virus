#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include "include/PCL_TEST_HEADER.h"

namespace fs = std::filesystem;

std::string folderPath = R"(C:/Users/tjut_/Desktop/20231123)";

void handle_file(const fs::path& path)
{
    // Step 1: First pass over the file to count lines
    std::ifstream infile_count_pass(path);
    if (!infile_count_pass) {
        std::cerr << "Failed to open input file: " << path << std::endl;
        return;
    }
    int point_count = std::count(std::istreambuf_iterator<char>(infile_count_pass),
                                 std::istreambuf_iterator<char>(), '\n');
    infile_count_pass.close();

    // Step 2: Process data
    std::ifstream infile(path);
    fs::path outpath = path;
    outpath.replace_extension(".pcd");
    std::ofstream outfile(outpath);
    if(!outfile.is_open())
    {
        std::cerr << "Failed to open output file: " << outpath << std::endl;
        return;
    }
    std::cout << "Processing: " << outpath << std::endl;

    // Write PCD header
    outfile << "# .PCD v.7 - Point Cloud Data file format\n"
            << "VERSION .7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4\n"
            << "TYPE F F F\n"
            << "COUNT 1 1 1\n"
            << "WIDTH " << point_count << "\n"
            << "HEIGHT 1\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS " << point_count << "\n"
            << "DATA ascii\n";

    std::string line;
    float x, y, z;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> x >> y >> z)
        {
            outfile << x << " " << y << " " << z << "\n";
        }
    }
}

int main()
{
    fs::path p(folderPath);
    if (!fs::exists(p) || !fs::is_directory(p)) {
        std::cerr << "Error: " << folderPath << " is not a valid directory." << std::endl;
        return 1;
    }

    for (const auto& entry : fs::recursive_directory_iterator(p))
    {
        if (fs::is_regular_file(entry) && entry.path().extension() == ".3dlp")
        {
            std::cout << "Processing file: " << entry.path() << std::endl;
            handle_file(entry.path());
        }
    }

    return 0;
}
