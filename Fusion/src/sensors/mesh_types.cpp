#include "pch.h"

#include <pcl/surface/marching_cubes_rbf.h>

template class pcl::PointCloud<pcl::PointNormal>;
template class pcl::search::KdTree<pcl::PointNormal>;
template class pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>;
template class pcl::GreedyProjectionTriangulation<pcl::PointNormal>;
template class pcl::MarchingCubesRBF<pcl::PointNormal>;
