#include "renderer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/random.hpp>

static const char* shader_vert =
#include "3d_shader.vert"
;

static const char* shader_frag =
#include "3d_shader.frag"
;

static const char* vert_point_src =
#include "point_shader.vert"
;

static const char* frag_point_src =
#include "point_shader.frag"
;

renderer::renderer(const view_window &window) :
    _window(window)
{
    _shader = std::shared_ptr<shader>(new shader(shader_frag, shader_vert));
    _point_shader = std::shared_ptr<shader>(new shader(frag_point_src, vert_point_src));

    view = glm::mat4();
    _proj = glm::perspective(glm::radians(45.0f), _window.width() / (float) _window.height(), 0.1f, 100.0f);

    float plane[] = {
        -50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        -50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
         50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
         50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };

    _shader->bind();
    _plane = std::shared_ptr<vbuffer>(new vbuffer(plane, 4 * 6, GL_TRIANGLE_STRIP));
    _plane->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 6);
    _plane->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 6);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    float tmp[] = { 0.0f, 0.0f, 1.0f, 0.0f };
    _point_shader->bind();
    _points = std::shared_ptr<vbuffer>(new vbuffer(tmp, 4, GL_POINTS));
    _points->setup_attrib(_point_shader->get_attrib_location("position"), 3, 0, 4);
    _points->setup_attrib(_point_shader->get_attrib_location("prob"), 1, 3, 4);

    _points_n = 1;
}

void renderer::update(float dt)
{
    (void) dt;
}

void renderer::render()
{
    _shader->bind();
    _shader->uniform_mat4("proj", _proj);
    _shader->uniform_mat4("model", view);
    _shader->uniform3f("eye", eye);
    _shader->uniform1i("light", light);

    _plane->bind();
    _shader->uniform3f("color", glm::vec3(0.95f, 0.95f, 0.95f));
    _plane->draw(0, 4);
    _plane->unbind();

    if(!points_mode) {
        // teehee
    }
    else {
        _point_shader->bind();
        _point_shader->uniform_mat4("proj", _proj);
        _point_shader->uniform_mat4("model", view);

        _points->bind();
        _points->draw(0, _points_n);
        _points->unbind();
    }
}

void renderer::upload_points(sensors& fetcher)
{
    sensors::cloud_res_t& res = fetcher.cloud();

    res.mutex().lock();
    size_t to_load = res.value().points().size();

    if(to_load == 0) {
        res.mutex().unlock();
        return;
    }

    float* buf = new float[4 * to_load];
    size_t i = 0;
    for(const point_t& p : res.value().points()) {
        buf[4 * i + 0] = p.x;
        buf[4 * i + 1] = p.y;
        buf[4 * i + 2] = p.z;
        buf[4 * i + 3] = p.p;
        i++;
    }
    res.mutex().unlock();

    _points_n = to_load;

    _points->bind();
    _points->update(buf, to_load * 4);

    delete[] buf;
}

void renderer::clear_points()
{
    _points->bind();
    _points->update(nullptr, 0);
}
