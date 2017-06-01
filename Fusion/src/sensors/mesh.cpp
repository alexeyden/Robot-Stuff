#include "mesh.h"

#include <pcl/surface/mls.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/poisson.h>

extern template class pcl::MarchingCubesRBF<pcl::PointNormal>;

bool mesh_builder::build(sensors::cloud_res_t &cloud_res, method m)
{
    if(_fut.valid())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Mesh builder: Filtering points..." << std::endl;
    cloud_res.mutex().lock();
    for(const auto& p : cloud_res.value().points()) {
        if(p.p > thresh) {
            cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
    }
    cloud_res.mutex().unlock();

    auto l = loc;

    auto labor = [cloud, m, l]() -> mesh_builder::data_t {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered { new pcl::PointCloud<pcl::PointXYZ> };

        std::cerr << "Mesh builder: outlier filtering...";
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> f(false);
        f.setInputCloud(cloud);
        f.setRadiusSearch(config::get<float>("mesh.filter.radius"));
        f.setMinNeighborsInRadius(config::get<int>("mesh.filter.min_nb"));
        f.filter(*cloud_filtered);

        std::cerr << "done: " << std::endl <<
                     "  filtered points n = " << cloud_filtered->points.size() << std::endl <<
                     "  original points n = " << cloud->points.size() << std::endl;

        std::vector<pcl::PointIndices> cluster_indices;
        std::cerr << "Mesh builder: clustering...";
        {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> extr;
            extr.setClusterTolerance(config::get<float>("mesh.cluster.tol"));
            extr.setMinClusterSize(config::get<int>("mesh.cluster.min_points"));
            extr.setMaxClusterSize(config::get<int>("mesh.cluster.max_points"));
            extr.setSearchMethod(tree);
            extr.setInputCloud(cloud_filtered);
            extr.extract(cluster_indices);
            std::cerr << "done: " << cluster_indices.size() << " clusters found" << std::endl;
        }

        mesh_builder::data_t vertices { new std::vector<float>() };

        int cluster_i = 0;
        for(const pcl::PointIndices& pi : cluster_indices) {
            std::cerr << "Mesh builder: processing cluster #" << cluster_i << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud { new pcl::PointCloud<pcl::PointXYZ> };
            for(int p : pi.indices) {
                cluster_cloud->push_back(cloud_filtered->points[p]);
            }

            pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree (new pcl::search::KdTree<pcl::PointXYZ>);
            cluster_tree->setInputCloud (cluster_cloud);

            std::cerr << "Mesh builder: normals estimation...";

              /*
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            n.setInputCloud (cluster_cloud);
            n.setSearchMethod (cluster_tree);
            n.setKSearch (config::get<int>("mesh.normals.k"));
            n.setViewPoint(l.x, l.y, l.z);
            n.compute (*normals);
            */

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            //pcl::concatenateFields (*cluster_cloud, *normals, *cloud_with_normals);

            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
            mls.setComputeNormals(true);
            mls.setInputCloud (cluster_cloud);
            mls.setPolynomialFit (false);
            mls.setSearchMethod (cluster_tree);
            mls.setSearchRadius (config::get<float>("mesh.normals.radius"));
            mls.process(*cloud_with_normals);


            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud (cloud_with_normals);

            std::cerr << "done" << std::endl;

            triang_result polygons;
            std::cerr << "Mesh builder: triangles..." << std::endl;
            if(m == method::GREEDY)
                polygons = greedy_triang(cloud_with_normals, tree2);
            else if(m == method::POISSON)
                polygons = poisson_triang(cloud_with_normals, tree2);
            else if(m == method::GRID_PROJ)
                polygons = marching_cubes_triang(cloud_with_normals, tree2);
            std::cerr << "Mesh builder: done" << std::endl;

            tree2.reset();

            std::cerr << "Mesh builder: conversion..." << std::endl;
            for(const pcl::Vertices& v : polygons.second) {
                if(v.vertices.size() != 3) {
                    std::cerr << "Mesh builder: vertices size != 3 (" << v.vertices.size() << ")" << std::endl;
                }
                const pcl::PointNormal& p1 = polygons.first->points[v.vertices[1]];
                const pcl::PointNormal& p2 = polygons.first->points[v.vertices[2]];
                const pcl::PointNormal& p3 = polygons.first->points[v.vertices[0]];

                glm::vec3 norm = glm::normalize(glm::cross(
                                                    glm::vec3(p2.x, p2.y, p2.z) - glm::vec3(p1.x, p1.y, p1.z),
                                                    glm::vec3(p3.x, p3.y, p3.z) - glm::vec3(p1.x, p1.y, p1.z)));

                float verts[] = {
                    p1.x, p1.y, p1.z, norm.x, norm.y, norm.z, cluster_i / (float) cluster_indices.size(),
                    p2.x, p2.y, p2.z, norm.x, norm.y, norm.z, cluster_i / (float) cluster_indices.size(),
                    p3.x, p3.y, p3.z, norm.x, norm.y, norm.z, cluster_i / (float) cluster_indices.size(),
                };
                vertices->insert(vertices->end(), verts, verts + 3 * 7);
            }
            std::cerr << "Mesh builder: done" << std::endl;
            cluster_i++;
        }

        std::cerr << "Mesh builder: all done" << std::endl;

        return vertices;
    };

    _fut = std::async(std::launch::async, labor);
    return true;
}

mesh_builder::triang_result mesh_builder::greedy_triang(
        const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
        const pcl::search::KdTree<pcl::PointNormal>::Ptr tree)
{
    std::cerr << "Mesh builder: greedy projection ..." << std::endl;

    std::vector<pcl::Vertices> polygons;

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> mc;
    mc.setSearchRadius (config::get<float>("mesh.surface.greedy.radius"));
    mc.setMu (config::get<float>("mesh.surface.greedy.mu"));
    mc.setMaximumNearestNeighbors (config::get<int>("mesh.surface.greedy.nbors"));
    mc.setMaximumSurfaceAngle(glm::radians(45.0f)); // 45 degrees
    mc.setMinimumAngle(glm::radians(10.0f)); // 10 degrees
    mc.setMaximumAngle(glm::radians(120.0f)); // 120 degrees
    mc.setNormalConsistency(false);

    mc.setInputCloud(cloud);
    mc.setSearchMethod(tree);
    mc.reconstruct(polygons);

    return std::make_pair(cloud, polygons);
}

mesh_builder::triang_result mesh_builder::poisson_triang(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree)
{
    std::cerr << "Mesh builder: poisson..." << std::endl;

    std::vector<pcl::Vertices> polygons;

    pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud { new pcl::PointCloud<pcl::PointNormal> };

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(cloud);
    poisson.setSearchMethod(tree);
    poisson.performReconstruction(*new_cloud, polygons);

    return std::make_pair(new_cloud, polygons);
}

mesh_builder::triang_result mesh_builder::marching_cubes_triang(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree)
{
    std::cerr << "Mesh builder: grid proj..." << std::endl;

    std::vector<pcl::Vertices> polygons;

    pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud { new pcl::PointCloud<pcl::PointNormal> };

    pcl::MarchingCubesHoppe<pcl::PointNormal> ps;
    ps.setIsoLevel(config::get<float>("mesh.surface.mcubes.iso"));
    //ps.setOffSurfaceDisplacement(config::get<float>("mesh.surface.mcubes.displ"));
    int res = config::get<int>("mesh.surface.mcubes.res");
    ps.setGridResolution(res, res, res);
    ps.setPercentageExtendGrid(config::get<float>("mesh.surface.mcubes.extend"));

    ps.setInputCloud(cloud);
    ps.setSearchMethod(tree);
    ps.reconstruct(*new_cloud, polygons);

    return std::make_pair(new_cloud, polygons);
}
