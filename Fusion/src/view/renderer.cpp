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

renderer::renderer(const view_window &window) :
    _window(window)
{
    _shader = std::shared_ptr<shader>(new shader(shader_frag, shader_vert));
    view = glm::mat4();
    _proj = glm::perspective(glm::radians(45.0f), _window.width() / (float) _window.height(), 0.1f, 100.0f);

    float plane[] = {
        -50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        -50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
         50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
         50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f

    };
    _plane = std::shared_ptr<vbuffer>(new vbuffer(plane, 4 * 6, GL_TRIANGLE_STRIP));
    _plane->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 6);
    _plane->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 6);
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

    _plane->bind();
    _shader->uniform3f("color", glm::vec3(0.95f, 0.95f, 0.95f));
    _plane->draw(0, 4);

    if(_points == nullptr)
        return;

    _shader->uniform3f("color", glm::vec3(0.2f, 0.8f, 0.3f));
    _points->bind();
    if(_points_buf.eptr() < _points_buf.size()) {
        _points->draw(0, _points_buf.eptr()/2);
    }
    else {
        _points->draw(0, _points_buf.fptr()/2);
        _points->draw(_points_buf.fptr()/2, _points_buf.eptr()/2 - _points_buf.fptr()/2);
    }
}

void renderer::upload_points(fetcher &fetcher)
{
    fetcher::laser_resource_t& res = fetcher.laser_points();
    res.mutex().lock();
    for(const glm::vec3& p : res.value()) {
        _points_buf.put(p);
        _points_buf.put(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
    }
    res.value().reset();
    res.mutex().unlock();

    if(_points == nullptr) {
        _points = std::shared_ptr<vbuffer>(new vbuffer((float*) _points_buf.data(),
                                                       _points_buf.eptr() * 3 * 2, GL_POINTS));
        _shader->bind();
        _points->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 6);
        _points->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 6);
    }
    else {
        _points->update((float*) _points_buf.data(), _points_buf.eptr() * 3 * 2);
    }
}
