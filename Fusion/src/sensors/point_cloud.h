#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#define PCL_NO_PRECOMPILE

#include <glm/glm.hpp>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 3 + 6 + 1 = 10
// 4 * 10 * 1 000 000 / 1024 / 1024 = 38 Mb per 1m

struct point_t {
    static constexpr size_t DIRN = 6;

    PCL_ADD_POINT4D

    float p;
    float pd[DIRN];

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class point_cloud
{
public:
    point_cloud();

    void add_laser(const glm::vec3& psensor, const glm::vec3& p);
    void add_usonic(const glm::vec3& psensor, const glm::vec3& p, float angle);

    const auto& points() const {
        return _cloud->points;
    }

    void clear();

private:
    boost::shared_ptr<pcl::PointCloud<point_t>> _cloud;
    pcl::octree::OctreePointCloudSearch<point_t> _tree;
};

#endif // POINT_CLOUD_H
