#ifndef PCL_TEST_CLUSTER_PROCESSOR_HPP
#define PCL_TEST_CLUSTER_PROCESSOR_HPP

#include "PCL_TEST_HEADER.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <map>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkPolygon.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Alpha_shape_vertex_base_3<K> Vb;
typedef CGAL::Alpha_shape_cell_base_3<K> Fb;
typedef CGAL::Triangulation_data_structure_3<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Triangulation_3;
typedef CGAL::Alpha_shape_3<Triangulation_3> Alpha_shape_3;
typedef K::Point_3 Point;
typedef Alpha_shape_3::FT FT;

struct ClusterMetrics
{
    int cluster_id = -1;
    int point_count = 0;
    std::vector<int> pointsID_set;
    float volume = 0;
    float density = 0;
    PointT centroid;
    std::array<uint8_t, 3> color; // 存储cluster的颜色
    std::vector<std::array<Point, 3>> alpha_shape_triangles; // 存储alpha shape的三角面片
};

class ClusterProcessor
{
public:
    static int process_clusters(MyPointCloud& cloud,
                                const std::string& folderPath,
                                const std::string& filename,
                                const std::tuple<float, float, float>& cloud_center,
                                const std::vector<int>& ordered_sequence,
                                const std::vector<float>& output_dist,
                                const float filter,
                                pcl::visualization::PCLVisualizer::Ptr& viewer,
                                const int minPointsFormingCluster = 4) {
        // 给点分配cluster ID
        const int clusterNumber = assign_cluster_ids(cloud, ordered_sequence, output_dist, filter,
                                                     minPointsFormingCluster);
        std::cout << "clusterNumber: (Start from 0) " << clusterNumber+1 << std::endl;

        // 只能通过共享指针的方式使得方法之间可以共享alphaShapes
        std::vector<std::shared_ptr<Alpha_shape_3>> alphaShapes;
        // 计算每个cluster的指标
        auto metrics = calculate_metrics(cloud, alphaShapes);

        // 根据metrics中存储的颜色给点着色
        apply_colors(cloud, cloud_center, metrics);

        // 可视化 facet
        visualizeAlphaShapes(cloud, cloud_center, metrics, viewer);

        // 保存结果
        save_results(cloud, folderPath, metrics);

        return clusterNumber+1;
    }

    static bool load_processed_cloud(MyPointCloud& cloud, const std::string& filepath) {
        if (pcl::io::loadPCDFile(filepath, cloud) == -1) {
            return false;
        }
        return true;
    }

private:
    static int assign_cluster_ids(MyPointCloud& cloud,
                                  const std::vector<int>& ordered_sequence,
                                  const std::vector<float>& output_dist,
                                  float filter,
                                  int minPointsFormingCluster) // did not filter the cluster less than 4 points
    {
        int clusterID_Index = -1;
        bool pre = true;
        bool cur = true;
        std::vector<int> cur_cluster;

        for (int i = 0; i < output_dist.size(); i++) {
            if (output_dist[i] < filter) {
                cur_cluster.push_back(ordered_sequence[i]); // 暂存当前的cluster id组
                cur = true;
                if (cur != pre) clusterID_Index += 1;
                if (i == 0) clusterID_Index += 1;
                cloud.points[ordered_sequence[i]].clusterID = clusterID_Index;
                pre = cur;
            }
            else {
                cur = false;
                cloud.points[ordered_sequence[i]].clusterID = -1;
                if (clusterID_Index == 77) {
                    std::cout << "Hello" << std::endl;
                };
                if (cur_cluster.size() >= 1 && cur_cluster.size() < minPointsFormingCluster) {
                    //cluster包含符合要求的点，但是数量不足以计算体积
                    for (auto j : cur_cluster) {
                        cloud.points[j].clusterID = -1; // 数量不足，全部设为noise
                    }
                    clusterID_Index -= 1; //回退cluster id
                }
                std::vector<int>().swap(cur_cluster); // empty vector and free space
                pre = cur;
            }
        }
        std::cout << "clusterID_MAX: " << clusterID_Index << std::endl;
        return clusterID_Index;
    }

    static std::vector<ClusterMetrics> calculate_metrics(const MyPointCloud& cloud,
                                                         std::vector<std::shared_ptr<Alpha_shape_3>>& alphaShapes) {
        std::vector<ClusterMetrics> metrics;
        std::map<int, std::vector<PointT>> clusters_points; // use dictionary to storage each cluster

        // 按cluster ID分组
        for (auto& point : cloud.points) {
            if (point.clusterID >= 0) {
                clusters_points[point.clusterID].push_back(point);
            }
        }

        // 计算每个cluster的指标
        // cluster set: {id:{points}}
        for (const auto& [fst, snd] : clusters_points) {
            // std::cout << fst << std::endl;

            ClusterMetrics metric;
            metric.cluster_id = fst;
            metric.point_count = snd.size();
            for (int i = 0; i < snd.size(); i++) {
                metric.pointsID_set.push_back(snd[i].pointID);
            }

            // 计算质心
            metric.centroid.x = metric.centroid.y = metric.centroid.z = 0;
            for (const auto& point : snd) {
                metric.centroid.x += point.x;
                metric.centroid.y += point.y;
                metric.centroid.z += point.z;
            }
            metric.centroid.x /= snd.size();
            metric.centroid.y /= snd.size();
            metric.centroid.z /= snd.size();

            // 生成随机颜色
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(50, 256);
            metric.color = {
                static_cast<uint8_t>(dis(gen)),
                static_cast<uint8_t>(dis(gen)),
                static_cast<uint8_t>(dis(gen))
            };

            // 计算体积（使用Alpha Shape）
            std::vector<Point> cgal_points;
            for (const auto& cgal_point : snd) {
                cgal_points.emplace_back(
                    cgal_point.x,
                    cgal_point.y,
                    cgal_point.z
                );
            }

            auto as_ptr = std::make_shared<Alpha_shape_3>(cgal_points.begin(), cgal_points.end());
            std::cout << "vertices number: " << as_ptr->number_of_vertices() << std::endl;
            alphaShapes.emplace_back(as_ptr);

            // 计算体积
            double volume = 0.0;
            std::cout << "Cluster ID: " << fst << std::endl;
            std::vector<Alpha_shape_3::Cell_handle> cells;
            Alpha_shape_3::NT alpha_solid = as_ptr->find_alpha_solid();
            as_ptr->set_alpha(alpha_solid);
            //as_ptr->set_alpha(100);

            if (as_ptr->number_of_solid_components() == 1) {
                std::cout << "Alpha value for single solid component: " << alpha_solid << std::endl;
            }
            else {
                std::cout << "find_alpha_solid() did not result in a single component." << std::endl;
            }
            as_ptr->get_alpha_shape_cells(std::back_inserter(cells), Alpha_shape_3::INTERIOR);
            std::cout << "Number of interior cells: " << cells.size() << std::endl;

            std::vector<Alpha_shape_3::Cell_handle> regular_cells, singular_cells;
            as_ptr->get_alpha_shape_cells(std::back_inserter(regular_cells), Alpha_shape_3::REGULAR);
            as_ptr->get_alpha_shape_cells(std::back_inserter(singular_cells), Alpha_shape_3::SINGULAR);

            // std::cout << "Regular cells: " << regular_cells.size() << std::endl;
            // std::cout << "Singular cells: " << singular_cells.size() << std::endl;
            //
            // std::cout << "Number of vertices in Alpha shape: " << as_ptr->number_of_vertices() << std::endl;
            // std::cout << "Number of edges in Alpha shape: " << as_ptr->number_of_solid_components() << std::endl;
            // std::cout << "Number of facets in Alpha shape: " << as_ptr->number_of_facets() << std::endl;
            // std::cout << "Number of cells in Alpha shape: " << as_ptr->number_of_solid_components() << std::endl;

            for (const auto& cell : cells) {
                volume += as_ptr->tetrahedron(cell).volume();
            }

            std::cout << "Volume: " << volume << std::endl;
            metric.volume = volume;

            // metric.volume = as.alpha_volume();
            if (metric.volume > 0) {
                metric.density = metric.point_count / metric.volume;
            }
            else if (metric.density == 0) {
                metric.density = 0;
            }

            // 存储外表面的三角形顶点
            if (as_ptr->number_of_vertices() < 4) {
                std::cout << "Abnormal facet found, vertices number less than 4" << std::endl;
            }
            else {
                for (Alpha_shape_3::Facet_iterator fit = as_ptr->facets_begin(); fit != as_ptr->facets_end(); ++fit) {
                    Alpha_shape_3::Classification_type type = as_ptr->classify(*fit);
                    if (type == Alpha_shape_3::REGULAR || type == Alpha_shape_3::SINGULAR) {
                        Alpha_shape_3::Cell_handle cell = fit->first;
                        int index = fit->second;

                        // 获取面的三个顶点
                        Point p1 = cell->vertex((index + 1) & 3)->point();
                        Point p2 = cell->vertex((index + 2) & 3)->point();
                        Point p3 = cell->vertex((index + 3) & 3)->point();

                        metric.alpha_shape_triangles.push_back({p1, p2, p3});
                    }
                }
            }
            //return final metrics
            metrics.push_back(metric);
        }

        std::cout << "Finish the part of calculate metrics." << std::endl;
        return metrics;
    }

    static void apply_colors(MyPointCloud& cloud, const std::tuple<float, float, float>& cloud_center, const std::vector<ClusterMetrics>& metrics) {
        // 先将所有点设为灰色（噪声点）
        for (auto& point : cloud.points) {
            if (point.clusterID < 0) {
                point.r = point.g = point.b = 50;
                point.a = 50;
            }
        }

        //给每个cluster的点着色
        for (const auto& metric : metrics) {
            for (auto& point : cloud.points) {
                if (point.clusterID == metric.cluster_id) {
                    point.r = metric.color[0];
                    point.g = metric.color[1];
                    point.b = metric.color[2];
                    // point.r = 255;
                    // point.g = 255;
                    // point.b = 255;
                }
            }
        }

        /** test the color of only one cluster **/
        for (auto& point : cloud.points) {
            if (point.clusterID ==3) {
                point.r = 255;
                point.g = 0;
                point.b = 0;
            }
            if (point.clusterID ==4) {
                point.r = 0;
                point.g = 255;
                point.b = 0;
            }
            if (point.clusterID ==5) {
                point.r = 0;
                point.g = 0;
                point.b = 255;
            }
        }

        // pcl::visualization::PCLVisualizer viewer("Cloud viewer"); // create a visualized window
        // viewer.addCoordinateSystem(1, std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center));
        // viewer.setBackgroundColor(0.1, 0.1, 0.1);
        // // pos: the position of camera, view: center of view, up: view direction
        // viewer.setCameraPosition(std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center), std::get<0>(cloud_center),
        //                          std::get<1>(cloud_center), 30, 0, 0, 0);
        //
        // // create a color processing object: PointCloudColorHandlerRGB
        // // PCLVisualizer Class use such object to display custom color data
        // // In this case, PointCloudColorHandlerRGB object can get the RGB value of each point
        // viewer.addPointCloud<PointT>(std::make_shared<MyPointCloud>(cloud), "sample cloud");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");
        //
        // while (!viewer.wasStopped())
        //     viewer.spinOnce(100);
    }

    static void visualizeAlphaShapes(const MyPointCloud& cloud,
                                     const std::tuple<float, float, float>& cloud_center,
                                     const std::vector<ClusterMetrics>& metrics,
                                     // const std::vector<std::shared_ptr<Alpha_shape_3>>& alphaShapes,
                                     pcl::visualization::PCLVisualizer::Ptr& viewer) {
        // 清除之前的所有点和形状
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // 直接添加 MyPointCloud 到可视化器
        viewer->addPointCloud<PointT>(std::make_shared<MyPointCloud>(cloud), "All points");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "All points");

        // 可视化每个alpha shape的三角面片
        for (const auto& cluster : metrics) {
            // 创建cluster标识符
            std::string cluster_id = "cluster_" + std::to_string(cluster.cluster_id);

            // 创建多边形网格
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();

            //添加所有alpha shape表面的顶点和三角面片
            std::map<Point, vtkIdType> point_id_map;
            // 对于每一个三角形
            for (const auto& triangle: cluster.alpha_shape_triangles) {
                vtkIdType ids[3];
                // 对于三角形的每一个顶点
                for (int i=0; i <3; i++ ) {
                    // 因为存在多个三角形共用顶点，去重
                    const auto& point = triangle[i];
                    if (point_id_map.find(point) == point_id_map.end()) { // 说明迭代器找到最后也没找到，元素不存在map中
                        // 双向label
                        vtkIdType id = points->InsertNextPoint(point[0], point[1], point[2]);
                        point_id_map[point] = id;
                    }
                    ids[i] = point_id_map[point];
                }

                triangles->InsertNextCell(3, ids); // 这里添加的只有id，后续可以反查from points
            }

            // 创建多边形
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            polydata->SetPoints(points);
            polydata->SetPolys(triangles);

            // 添加到可视化器
            viewer->addModelFromPolyData(polydata, cluster_id);

            // 设置颜色
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                cluster.color[0]/255.0, cluster.color[1]/255.0, cluster.color[2]/255.0,
                cluster_id);

            // 可选：设置透明度
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, cluster_id);
        }



        // 设置背景颜色和相机位置
        viewer->addCoordinateSystem(1, std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center));
        viewer->setBackgroundColor(0.1, 0.1, 0.1);
        viewer->setCameraPosition(std::get<0>(cloud_center), std::get<1>(cloud_center), std::get<2>(cloud_center), std::get<0>(cloud_center),
                                  std::get<1>(cloud_center), 30, 0, 0, 0);

        while (!viewer->wasStopped())
            viewer->spinOnce(100);
    }

    static void save_results(const MyPointCloud& cloud, const std::string& folderPath,
                             const std::vector<ClusterMetrics>& metrics) {
        // 保存点云
        pcl::io::savePCDFile(folderPath + "clustered_cloud.pcd", cloud);

        // 保存指标
        std::ofstream metrics_file(folderPath + "/cluster_metrics.csv");
        metrics_file << "Cluster ID,Point Count,Volume,Density,Centroid X,Centroid Y,Centroid Z\n";
        for (const auto& metric : metrics) {
            metrics_file << metric.cluster_id << ","
                << metric.point_count << ","
                << metric.volume << ","
                << metric.density << ","
                << metric.centroid.x << ","
                << metric.centroid.y << ","
                << metric.centroid.z << "\n";
        }
        metrics_file.close();
    }
};

#endif //PCL_TEST_CLUSTER_PROCESSOR_HPP
