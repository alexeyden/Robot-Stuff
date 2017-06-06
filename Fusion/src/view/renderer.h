#ifndef RENDERER_H
#define RENDERER_H

#include <memory>
#include <glm/glm.hpp>

#include "sensors/sensors.h"
#include "sensors/crbuffer.h"

#include "view_window.h"
#include "vbuffer.h"
#include "shader.h"
#include "texture.h"

class renderer
{
public:
    renderer(const view_window& window);

    void update(float dt);
    void resize();
    void render();

    bool upload_points(sensors& sensors);
    void upload_mesh(const std::vector<float>& data);
    void update_objects(sensors& sensors);

    glm::mat4 view;
    glm::vec3 eye;
    glm::vec3 dir;
    glm::vec3 light_pos;
    glm::vec3 light_target;

    size_t points_count() const {
        return _points_n;
    }

    size_t vertex_count() const {
        return _verts_n;
    }

    size_t objects_count() const {
        return _objects.size();
    }

    void clear_points();

    bool light = true;
    bool points_mode = true;
    bool ground_plane = true;
    bool shadow = true;
    float thresh;

    GLuint shadow_tex() const {
        return _shadow_map;
    }

    std::shared_ptr<vbuffer> mesh() {
        return _mesh;
    }

private:
    void init_shadows();

    glm::mat4 _proj;

    size_t _points_n;
    size_t _verts_n;

    GLuint _fbo;
    GLuint _shadow_map;

    std::shared_ptr<shader> _shader;
    std::shared_ptr<shader> _point_shader;
    std::shared_ptr<shader> _shadow_shader;

    std::shared_ptr<vbuffer> _points;
    std::shared_ptr<vbuffer> _plane;
    std::shared_ptr<vbuffer> _mesh;

    std::shared_ptr<vbuffer> _obj_mesh;

    std::vector<glm::mat4> _objects;

    const view_window& _window;

    float _t = 0;
};

#endif // RENDERER_H
