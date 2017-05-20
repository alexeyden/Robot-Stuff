#ifndef RENDERER_H
#define RENDERER_H

#include <memory>
#include <glm/glm.hpp>

#include "sensors/sensors.h"
#include "sensors/crbuffer.h"

#include "view_window.h"
#include "vbuffer.h"
#include "shader.h"

class renderer
{
public:
    renderer(const view_window& window);

    void update(float dt);
    void resize();
    void render();

    void upload_points(sensors& sensors);
    void upload_mesh(const std::vector<float>& data);

    glm::mat4 view;
    glm::vec3 eye;
    glm::vec3 dir;

    size_t points_count() const {
        return _points_n;
    }

    size_t vertex_count() const {
        return _verts_n;
    }

    void clear_points();

    bool light = true;
    bool points_mode = true;
    bool ground_plane = true;
    float thresh;
private:
    glm::mat4 _proj;

    size_t _points_n;
    size_t _verts_n;

    std::shared_ptr<shader> _shader;
    std::shared_ptr<shader> _point_shader;
    std::shared_ptr<vbuffer> _points;
    std::shared_ptr<vbuffer> _plane;
    std::shared_ptr<vbuffer> _mesh;

    const view_window& _window;
};

#endif // RENDERER_H
