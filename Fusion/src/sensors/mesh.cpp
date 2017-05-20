#include "mesh.h"

#include <pcl/surface/marching_cubes_rbf.h>

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
        std::cerr << "Mesh builder: normals..." << std::endl;

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);
        n.setInputCloud (cloud);
        n.setSearchMethod (tree);
        n.setKSearch (25);
        n.setViewPoint(l.x, l.y, l.z);
        n.compute (*normals);

        std::cerr << "Mesh builder: done" << std::endl;

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

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
        mesh_builder::data_t vertices { new std::vector<float>() };
        for(const pcl::Vertices& v : polygons.second) {
            if(v.vertices.size() != 3) {
                std::cerr << "Mesh builder: vertices size != 3 (" << v.vertices.size() << ")" << std::endl;
            }
            const pcl::PointNormal& p1 = polygons.first->points[v.vertices[1]];
            const pcl::PointNormal& p2 = polygons.first->points[v.vertices[2]];
            const pcl::PointNormal& p3 = polygons.first->points[v.vertices[0]];

	    /*
            glm::vec3 norm = glm::normalize(glm::cross(
                        glm::vec3(p2.x, p2.y, p2.z) - glm::vec3(p1.x, p1.y, p1.z),
                        glm::vec3(p3.x, p3.y, p3.z) - glm::vec3(p1.x, p1.y, p1.z)));

            float verts[] = {
                p1.x, p1.y, p1.z, norm.x, norm.y, norm.z,
                p2.x, p2.y, p2.z, norm.x, norm.y, norm.z,
                p3.x, p3.y, p3.z, norm.x, norm.y, norm.z
            };
	    */
	    glm::vec3 norm = glm::normalize(glm::vec3(
				       (p1.x + p2.x + p3.x)/3.0f, 
				       (p1.y + p2.y + p3.y)/3.0f, 
				       (p1.z + p2.z + p3.z)/3.0f)); 

            float verts[] = {
                p1.x, p1.y, p1.z, norm.x, norm.y, norm.z,
                p2.x, p2.y, p2.z, norm.x, norm.y, norm.z,
                p3.x, p3.y, p3.z, norm.x, norm.y, norm.z
            };
            vertices->insert(vertices->end(), verts, verts + 3 * 6);
        }
        std::cerr << "Mesh builder: done" << std::endl;

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
    mc.setSearchRadius (2.0f);
    mc.setMu (2.5);
    mc.setMaximumNearestNeighbors (100);
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

    return std::make_pair(new_cloud, polygons);
}

mesh_builder::triang_result mesh_builder::marching_cubes_triang(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree)
{
    std::cerr << "Mesh builder: grid proj..." << std::endl;

    std::vector<pcl::Vertices> polygons;

    pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud { new pcl::PointCloud<pcl::PointNormal> };

    pcl::MarchingCubesRBF<pcl::PointNormal> ps;
    ps.setIsoLevel(0);
    ps.setOffSurfaceDisplacement(0.01f);
    ps.setGridResolution(30, 30, 30);
    ps.setPercentageExtendGrid(0.0f);

    ps.setInputCloud(cloud);
    ps.setSearchMethod(tree);
    ps.reconstruct(*new_cloud, polygons);

    return std::make_pair(new_cloud, polygons);
}
