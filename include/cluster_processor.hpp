#ifndef PCL_TEST_CLUSTER_PROCESSOR_HPP
#define PCL_TEST_CLUSTER_PROCESSOR_HPP

#include "PCL_TEST_HEADER.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <map>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Alpha_shape_vertex_base_3<K> Vb;
typedef CGAL::Alpha_shape_cell_base_3<K> Fb;
typedef CGAL::Triangulation_data_structure_3<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Triangulation_3;
typedef CGAL::Alpha_shape_3<Triangulation_3> Alpha_shape_3;
typedef K::Point_3 Point;

struct ClusterMetrics
{
    int cluster_id;
    int point_count;
    float volume=0;
    float density=0;
    PointT centroid;
    std::array<uint8_t, 3> color; // 存储cluster的颜色
    std::vector<std::array<Point, 3>> alpha_shape_triangles; // 存储alpha shape的三角面片
};

class ClusterProcessor
{
public:
    static int process_clusters(MyPointCloud& cloud,
                                const std::string& folderPath,
                                const std::vector<int>& ordered_sequence,
                                const std::vector<float>& output_dist,
                                float filter,
                                int minPointsFormingCluster=4)
    {
        // 给点分配cluster ID
        int clusterNumber = assign_cluster_ids(cloud, ordered_sequence, output_dist, filter, minPointsFormingCluster);

        // 计算每个cluster的指标
        auto metrics = calculate_metrics(cloud);

        // 根据metrics中存储的颜色给点着色
        apply_colors(cloud, metrics);

        // 保存结果
        save_results(cloud, folderPath, metrics);

        // 创建可视化器
        pcl::visualization::PCLVisualizer viewer("Alpha Shape Visualization");
        viewer.setBackgroundColor(0.1, 0.1, 0.1);

        // 添加原始点云
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud.makeShared());
        viewer.addPointCloud<PointT>(cloud.makeShared(), rgb, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

        // 为每个cluster添加alpha shape mesh
        int mesh_id = 0;
        for (const auto& metric : metrics)
        {
            for (const auto& triangle : metric.alpha_shape_triangles)
            {
                std::string id = "triangle_" + std::to_string(mesh_id++);
                viewer.addPolygon<pcl::PointXYZ>(
                    pcl::PointXYZ(triangle[0].x(), triangle[0].y(), triangle[0].z()),
                    pcl::PointXYZ(triangle[1].x(), triangle[1].y(), triangle[1].z()),
                    pcl::PointXYZ(triangle[2].x(), triangle[2].y(), triangle[2].z()),
                    metric.color[0]/255.0, metric.color[1]/255.0, metric.color[2]/255.0,
                    id
                );
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id);
            }
        }

        // 显示可视化结果
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }

        return clusterNumber;
    }

    static bool load_processed_cloud(MyPointCloud& cloud, const std::string& filepath)
    {
        if (pcl::io::loadPCDFile(filepath, cloud) == -1)
        {
            return false;
        }
        return true;
    }

private:
    static int assign_cluster_ids(MyPointCloud& cloud,
                                  const std::vector<int>& ordered_sequence,
                                  const std::vector<float>& output_dist,
                                  float filter, int minPointsFormingCluster) // did not filter the cluster less than 3 points
    {
        int clusterID_Index = 0;
        bool pre = true;
        bool cur = true;
        std::vector<int> cur_cluster;

        for (int i = 0; i < output_dist.size(); i++)
        {
            if (output_dist[i] < filter)
            {
                cur_cluster.push_back(ordered_sequence[i]); // 暂存当前的cluster id组
                cur = true;
                if (cur != pre) clusterID_Index += 1;
                if (i == 0) clusterID_Index += 1;
                cloud.points[ordered_sequence[i]].clusterID = clusterID_Index;
                pre = cur;
            }
            else
            {
                cur = false;
                cloud.points[ordered_sequence[i]].clusterID = -1;
                if (cur_cluster.size() >=1 && cur_cluster.size() < minPointsFormingCluster) //cluster包含符合要求的点，但是数量不足以计算体积
                {
                    for (auto j : cur_cluster)
                    {
                        cloud.points[ordered_sequence[j]].clusterID = -1; // 数量不足，全部设为noise
                    }
                    clusterID_Index -= 1; //回退cluster id
                }
                std::vector<int>().swap(cur_cluster); // empty vector and free space
                pre = cur;
            }
        }
        return clusterID_Index;
    }

    static std::vector<ClusterMetrics> calculate_metrics(const MyPointCloud& cloud)
    {
        std::vector<ClusterMetrics> metrics;
        std::map<int, std::vector<PointT>> clusters_points; // use dictionary to storage each cluster

        // 按cluster ID分组
        for (auto& point : cloud.points)
        {
            if (point.clusterID >= 0)
            {
                clusters_points[point.clusterID].push_back(point);
            }
        }

        // 计算每个cluster的指标
        // cluster set: {id:{points}}
        for (const auto& pair : clusters_points)
        {
            ClusterMetrics metric;
            metric.cluster_id = pair.first;
            metric.point_count = pair.second.size();

            // 计算质心
            metric.centroid.x = metric.centroid.y = metric.centroid.z = 0;
            for (const auto& point : pair.second)
            {
                metric.centroid.x += point.x;
                metric.centroid.y += point.y;
                metric.centroid.z += point.z;
            }
            metric.centroid.x /= pair.second.size();
            metric.centroid.y /= pair.second.size();
            metric.centroid.z /= pair.second.size();
            
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
            for (const auto& cgal_point : pair.second)
            {
                cgal_points.push_back(Point(
                    cgal_point.x,
                    cgal_point.y,
                    cgal_point.z
                ));
            }

            Alpha_shape_3 as(cgal_points.begin(), cgal_points.end());
            auto optimal_alpha = as.find_optimal_alpha(1); // 容易出错
            as.set_alpha(*optimal_alpha);

            // 获取alpha shape的面片用于可视化
            std::vector<Alpha_shape_3::Facet> facets;
            as.get_alpha_shape_facets(std::back_inserter(facets), Alpha_shape_3::REGULAR);
            
            // 将面片添加到PCL可视化器
            for (const auto& facet : facets)
            {
                // 确保面片方向一致
                if (as.classify(facet.first) != Alpha_shape_3::EXTERIOR)
                    facet = as.mirror_facet(facet);
                
                // 获取三角形的三个顶点
                int indices[3] = {
                    (facet.second + 1) % 4,
                    (facet.second + 2) % 4,
                    (facet.second + 3) % 4
                };
                if (facet.second % 2 == 0) 
                    std::swap(indices[0], indices[1]);
                    
                // 存储三角形顶点坐标
                metric.alpha_shape_triangles.push_back({
                    facet.first->vertex(indices[0])->point(),
                    facet.first->vertex(indices[1])->point(),
                    facet.first->vertex(indices[2])->point()
                });
            }


            // 计算体积
            double volume = 0.0;
            for(Alpha_shape_3::Cell_iterator cit = as.finite_cells_begin(); cit != as.finite_cells_end(); ++cit) 
            {
                if(as.classify(cit) == Alpha_shape_3::INTERIOR) {
                    K::Tetrahedron_3 t = K().construct_tetrahedron_3_object()(
                        cit->vertex(0)->point(),
                        cit->vertex(1)->point(),
                        cit->vertex(2)->point(),
                        cit->vertex(3)->point()
                    );
                    volume += t.volume();
                }
            }

            std::cout << "Volume: " << volume << std::endl;
            metric.volume = volume;


            // metric.volume = as.alpha_volume();
            if (metric.volume > 0)
            {
                metric.density = metric.point_count / metric.volume;
            }

            metrics.push_back(metric);
        }

        return metrics;
    }

    static void apply_colors(MyPointCloud& cloud, const std::vector<ClusterMetrics>& metrics)
    {
        // 先将所有点设为灰色（噪声点）
        for (auto& point : cloud.points)
        {
            if (point.clusterID < 0)
            {
                point.r = point.g = point.b = 50;
                point.a = 50;
            }
        }

        // 给每个cluster的点着色
        for (const auto& metric : metrics)
        {
            for (auto& point : cloud.points)
            {
                if (point.clusterID == metric.cluster_id)
                {
                    point.r = metric.color[0];
                    point.g = metric.color[1];
                    point.b = metric.color[2];
                }
            }
        }
    }

    static void save_results(const MyPointCloud& cloud, const std::string& folderPath,
                             const std::vector<ClusterMetrics>& metrics)
    {
        // 保存点云
        pcl::io::savePCDFile(folderPath + "clustered_cloud.pcd", cloud);

        // 保存指标
        std::ofstream metrics_file(folderPath + "/cluster_metrics.csv");
        metrics_file << "Cluster ID,Point Count,Volume,Density,Centroid X,Centroid Y,Centroid Z\n";
        for (const auto& metric : metrics)
        {
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
