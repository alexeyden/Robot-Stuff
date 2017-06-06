#include "point_cloud.h"

#include <fstream>

point_cloud::point_cloud()
{
    _cloud = boost::shared_ptr<pcloud_t>(new pcloud_t());
    _tree = boost::shared_ptr<tree_t>(new tree_t(0.1f));
    _tree->setInputCloud(_cloud);
}

void point_cloud::add_laser(const glm::vec3 &psensor, const glm::vec3 &p)
{
    point_t point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.p = 0.5f;
    std::fill(point.pd, point.pd + point.SUB_N, 1.0f - std::pow(0.5, 1.0/float(point.SUB_N)));

    std::vector<int> indices;
    std::vector<float> distances;

    if(_tree->radiusSearch(point, _eps, indices, distances) > 0) {
        for(int i : indices) {
            update_point(_cloud->points[i], psensor, 1.0f);
        }
    }
    else {
        _tree->addPointToCloud(point, _cloud);
    }
}

void point_cloud::add_usonic(const glm::vec3 &psensor, const glm::vec3 &p, float angle, bool blackout)
{
    glm::vec3 dr = p - psensor;
    float d = glm::length(dr);
    dr = glm::normalize(dr);

    usonic_blackout(psensor, p, angle);

    if(blackout)
        return;

    const int suba = 20;
    const int subb = 20;
    const float da = angle/2.0f/suba;
    const float db = glm::radians(360.0f)/subb;

    std::vector<int> indices;
    std::vector<float> distances;

    // TODO: proper cone vs plane intersection
    for(float a = angle/2.0f; a > 0; a -= da) {
        glm::mat4 r = glm::rotate(glm::mat4(), a, glm::normalize(glm::cross(dr, glm::vec3(-dr[1], dr[2], dr[0]))));
        for(float s = 0; s < glm::radians(360.0f); s += db) {
            glm::vec4 pp = glm::rotate(glm::mat4(), s, dr) * r * glm::vec4(dr, 1.0f);

            point_t point;
            point.z = pp.z * d + psensor.z;

            if(point.z < 0.45f)
                return;
        }
    }

    for(float a = angle/2.0f; a > 0; a -= da) {
        glm::mat4 r = glm::rotate(glm::mat4(), a, glm::normalize(glm::cross(dr, glm::vec3(-dr[1], dr[2], dr[0]))));
        for(float s = 0; s < glm::radians(360.0f); s += db) {
            glm::vec4 pp = glm::rotate(glm::mat4(), s, dr) * r * glm::vec4(dr, 1.0f);

            point_t point;
            point.x = pp.x * d + psensor.x;
            point.y = pp.y * d + psensor.y;
            point.z = pp.z * d + psensor.z;

            const float S = 2.0f * glm::pi<float>() * d * d * (1.0f - std::cos(angle/2.0f));
            const float k = std::max(0.5f, 1.0f/S);

            if(_tree->radiusSearch(point, _eps, indices, distances) > 0) {
                for(int ind : indices) {
                    update_point(_cloud->points[ind], psensor, k);
                }
            } else {
                point.p = 0.5f;
                std::fill(point.pd, point.pd + point.SUB_N, 1.0f - std::pow(0.5, 1.0/float(point.SUB_N)));

                update_point(point, psensor, k);

                _tree->addPointToCloud(point, _cloud);
            }
        }
    }
}

void point_cloud::clear()
{
    _cloud->clear();
    _tree->deleteTree();
}

void point_cloud::load(const char *path)
{
    clear();
    std::ifstream fs(path);

    point_t p;
    while(!fs.eof()) {
        fs >> p.x >> p.y >> p.z >> p.p;

        for(int i = 0; i < point_t::SUB_N; i++)
            fs >> p.pd[i];

        if(!fs.eof()) {
            _tree->addPointToCloud(p, _cloud);
        }
    }
}

void point_cloud::save(const char *p)
{
    std::ofstream fs(p);
    for(const point_t& p : _cloud->points) {
        fs << p.x << " " << p.y << " " << p.z << " " << p.p;

        for(int i = 0; i < point_t::SUB_N; i++)
            fs << " " << p.pd[i];

        fs << std::endl;
    }
}

uint32_t point_cloud::get_dir(const glm::vec3& src, const glm::vec3& dst, uint32_t subxy, uint32_t subz)
{
    // get direction index in lat/long subdivision

    glm::vec3 dir = glm::normalize(dst - src);
    uint32_t xyi = std::min<uint32_t>((std::atan2(dir.y, dir.x) + glm::pi<float>())
                                      / (glm::pi<float>() * 2) * subxy, subxy - 1);
    uint32_t xzi = std::min<uint32_t>((dir.z + 1.0f)/2.0f * subz, subz - 1);

    return 0;
}

void point_cloud::update_point(point_t &p, const glm::vec3 &src, float k)
{
    glm::vec3 glm_p(p.x, p.y, p.z);
    uint32_t di = get_dir(src, glm_p, p.SUB_LONG, p.SUB_LAT);

    float ps = std::min(1.0f, k);
    float pp = p.pd[di];
    float pn = std::min(ps * pp / (ps * pp + (1.0f - ps) * (1.0f - pp) + 0.001f), 1.0f);

    p.pd[di] = pn;

    float po = 1.0f;
    for(uint32_t i = 0; i < p.SUB_N; i++)
        po *= (1.0f - p.pd[i]);

    po = 1.0f - po;
    p.p = po;
}

void point_cloud::usonic_blackout(const glm::vec3 &src, const glm::vec3 &dst, float angle)
{
    const glm::vec3 dir = dst - src;
    const float d = glm::length(dir);
    const float alpha = std::atan2(dir.y, dir.x);
    const float beta = std::atan2(dir.z, glm::length(glm::vec2(dir)));

    glm::vec3 oa {
        d * std::cos(alpha + angle/2),
        d * std::sin(alpha + angle/2),
        d * std::sin(beta + angle/2)
    };

    glm::vec3 ob {
        d * std::cos(alpha - angle/2),
        d * std::sin(alpha - angle/2),
        d * std::sin(beta - angle/2)
    };

    glm::vec3 oc {
        d * std::cos(alpha),
        d * std::sin(alpha),
        d * std::sin(beta)
    };

    Eigen::Vector3f min;
    Eigen::Vector3f max;

    min(0) = glm::compMin(glm::vec4(0.0f, oa.x, ob.x, oc.x)) + src.x;
    min(1) = glm::compMin(glm::vec4(0.0f, oa.y, ob.y, oc.y)) + src.y;
    min(2) = glm::compMin(glm::vec4(0.0f, oa.z, ob.z, oc.z)) + src.z;

    max(0) = glm::compMax(glm::vec4(0.0f, oa.x, ob.x, oc.x)) + src.x;
    max(1) = glm::compMax(glm::vec4(0.0f, oa.y, ob.y, oc.y)) + src.y;
    max(2) = glm::compMax(glm::vec4(0.0f, oa.z, ob.z, oc.z)) + src.z;

    std::vector<int> indices;
    _tree->boxSearch(min, max, indices);

    for(int i : indices) {
        point_t& point = _cloud->points[i];
        glm::vec3 pgl = glm::vec3 { point.x, point.y, point.z } - src;
        float p_dist = glm::length(pgl);

        if (glm::dot(glm::normalize(dir), glm::normalize(pgl)) > std::cos(angle/2.0f) && (d - p_dist) > _eps) {
            update_point(point, src, _bop);
        }
    }
}
