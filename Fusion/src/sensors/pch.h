#ifndef PCH_H
#define PCH_H

#include <thread>
#include <iostream>
#include <cstring>
#include <iostream>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <future>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/component_wise.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "config.h"

#endif // PCH_H
