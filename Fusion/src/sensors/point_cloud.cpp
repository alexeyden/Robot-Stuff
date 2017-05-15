#include "point_cloud.h"


point_cloud::point_cloud() :
    _tree(0.1)
{
    _cloud = boost::shared_ptr<pcl::PointCloud<point_t>>(new pcl::PointCloud<point_t>());
    _tree.setInputCloud(_cloud);
}

void point_cloud::add_laser(const glm::vec3 &psensor, const glm::vec3 &p)
{
    point_t point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.p = (std::rand() % 10) / 10.0f;
    _tree.addPointToCloud(point, _cloud);
}

void point_cloud::add_usonic(const glm::vec3 &psensor, const glm::vec3 &p, float angle)
{

}

void point_cloud::clear()
{
    _cloud->clear();
    _tree.deleteTree();
}
