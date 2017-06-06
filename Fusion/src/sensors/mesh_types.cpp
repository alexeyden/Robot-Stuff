#include "pch.h"

//force instantiation to cut down build times

template class pcl::PointCloud<pcl::PointNormal>;
template class pcl::search::KdTree<pcl::PointNormal>;
template class pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>;
template class pcl::GreedyProjectionTriangulation<pcl::PointNormal>;
template class pcl::MarchingCubesHoppe<pcl::PointNormal>;
