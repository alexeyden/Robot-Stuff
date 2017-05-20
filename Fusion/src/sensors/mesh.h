#ifndef MESH_H
#define MESH_H

#include "pch.h"
#include "sensors.h"
#include <utility>

extern template class pcl::PointCloud<pcl::PointNormal>;
extern template class pcl::search::KdTree<pcl::PointNormal>;
extern template class pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>;
extern template class pcl::GreedyProjectionTriangulation<pcl::PointNormal>;

class mesh_builder {
public:
    enum class method {
        GREEDY,
        POISSON,
        GRID_PROJ
    };

    mesh_builder() {}

    bool build(sensors::cloud_res_t& cloud_res, method m);

    typedef std::shared_ptr<std::vector<float>> data_t;

    std::future<data_t>& result() {
        return _fut;
    }

    void reset() {
        _fut = std::future<data_t>();
    }

    float thresh = 0.6f;
    glm::vec3 loc;

private:
    typedef std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, std::vector<pcl::Vertices>> triang_result;

    static triang_result greedy_triang(
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree);

    static triang_result poisson_triang(
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree);

    static triang_result marching_cubes_triang(
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree);
    std::future<data_t> _fut;
};

#endif // MESH_H
