#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "pch.h"

// 3 + 6 + 1 = 10
// 4 * 10 * 1 000 000 / 1024 / 1024 = 38 Mb per 1m

struct point_t {
    static constexpr size_t SUB_LONG = 1;
    static constexpr size_t SUB_LAT = 1;
    static constexpr size_t SUB_N = SUB_LONG * SUB_LAT;

    PCL_ADD_POINT4D

    float p;
    float pd[SUB_LONG * SUB_LAT];

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class point_cloud
{
public:
    point_cloud();

    void add_laser(const glm::vec3& psensor, const glm::vec3& p);
    void add_usonic(const glm::vec3& psensor, const glm::vec3& p, float angle, bool blackout = false);

    const auto& points() const {
        return _cloud->points;
    }

    void clear();
    void load(const char* p);
    void save(const char* p);

    typedef pcl::PointCloud<point_t> pcloud_t;
    typedef pcl::octree::OctreePointCloudSearch<point_t> tree_t;

    const boost::shared_ptr<pcloud_t>& cloud() const {
        return _cloud;
    }

    const boost::shared_ptr<tree_t>& tree() const {
        return _tree;
    }
private:
    static constexpr float _eps = 0.1f;
    static constexpr float _bop = 0.01f;

    uint32_t get_dir(const glm::vec3& src, const glm::vec3 &dst, uint32_t subxy, uint32_t subz);

    void update_point(point_t& p, const glm::vec3& src, float k);
    void usonic_blackout(const glm::vec3& src, const glm::vec3& dst, float angle);

    boost::shared_ptr<pcloud_t> _cloud;
    boost::shared_ptr<tree_t> _tree;
};

#endif // POINT_CLOUD_H
