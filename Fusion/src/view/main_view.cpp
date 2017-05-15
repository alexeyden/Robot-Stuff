#include "main_view.h"

#include <cstdio>
#include <fstream>
#include <iostream>

#include <glm/gtc/matrix_transform.hpp>

static const char* vert_2d_src =
#include "2d_shader.vert"
;

static const char* frag_2d_src =
#include "2d_shader.frag"
;

main_view::main_view(view_window *parent) :
    view(parent), _renderer(*parent)
{
    _2d_shader = std::shared_ptr<shader>(new shader(frag_2d_src, vert_2d_src));
    _font = std::shared_ptr<font>(new font("data/font.png", _2d_shader, 2.0f, 8, 8));

    _font_proj = glm::ortho(0.0f, (float) parent->width(), (float) parent->height(), 0.0f);

    _time = 0.0f;

    _pos = glm::vec3(0.0f, 0.0f, 1.0f);
    _dir = glm::vec3(0.0f, 1.0f, 0.0f);
    _up = glm::vec3(0.0f, 0.0f, 1.0f);

    _angle_h = 0.0f;
    _angle_v = 0.0f;

    _cursor = true;
    _light = true;
}

void main_view::update(float dt)
{
    _time += dt;

    if(_time > 2.0f) {
        _renderer.upload_points(_window->sensors_data());
        _time = 0.0f;
    }

    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_W) == GLFW_PRESS) {
        _pos -= _dir * dt * 10.0f;
    }
    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_S) == GLFW_PRESS) {
        _pos += _dir * dt * 10.0f;
    }
    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_A) == GLFW_PRESS) {
        _pos -= glm::normalize(glm::cross(_up, _dir)) * dt * 5.0f;
    }
    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_D) == GLFW_PRESS) {
        _pos += glm::normalize(glm::cross(_up, _dir)) * dt * 5.0f;
    }

    _renderer.update(dt);
}

void main_view::draw()
{
    static char buf[512];

    _renderer.view = glm::lookAt(_pos, _pos - _dir, _up);
    _renderer.eye = _pos;
    _renderer.light = _light;
    _renderer.render();

    _2d_shader->bind();
    _2d_shader->uniform_mat4("proj", _font_proj);

     snprintf(buf, 511, "pos: %.2f %.2f %.2f\n"
                        "dir: %.2f %.2f %2f\n"
                        "fps: %.1f\n"
                        "pts_n: %lu\n"
                        "pause: %d",
             _pos.x, _pos.y, _pos.z,
             _dir.x, _dir.y, _dir.z,
             _window->fps(),
             _renderer.points_count(),
             _window->sensors_data().pause);

    _font->draw((uint8_t*) "MAIN VIEW", 10, 10);
    _font->draw((uint8_t*) buf, 10, 24);

    _font->draw((uint8_t*) "Keys:\n"
                           "c - dump point cloud\n"
                           "l - lighting on\\off\n"
                           "k - nuke cloud\n"
                           "p - pause updates", 10, 60);

    if(!_cursor) {
        _font->draw((uint8_t*) "\x07", _window->width()/2 - 4, _window->height()/2 - 4);
    }

}

void main_view::on_mouse(float x, float y)
{
    if(_mouse.x != 0.0f && _mouse.y != 0.0f && !_cursor) {
        float dx = x - _mouse.x;
        float dy = y - _mouse.y;

        _angle_v -= dy * 0.01;
        _angle_h -= dx * 0.01;

        glm::vec3 dir_axis(0.0f, 1.0f, 0.0f);
        glm::vec3 up_axis(0.0f, 0.0f, 1.0f);

        auto rot = glm::rotate(glm::mat4(), _angle_h, up_axis);
        dir_axis = glm::normalize(rot * glm::vec4(dir_axis, 1.0f));

        rot = glm::rotate(glm::mat4(), _angle_v, glm::normalize(glm::cross(up_axis, dir_axis)));
        dir_axis = glm::normalize(rot * glm::vec4(dir_axis, 1.0f));

        _dir = dir_axis;
        _up = glm::normalize(glm::cross(dir_axis, glm::cross(up_axis, dir_axis)));
    }

    _mouse = glm::vec2(x,y);
}

void main_view::on_key(int k, bool r)
{
    if(k == GLFW_KEY_SPACE && r && _cursor) {
        glfwSetInputMode(_window->glfw_window(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        _cursor = false;
    }
    else if(k == GLFW_KEY_SPACE && r && !_cursor) {
        glfwSetInputMode(_window->glfw_window(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        _cursor = true;
    }
    else if(k == GLFW_KEY_L && r) {
        _light = !_light;
    }
    else if(k == GLFW_KEY_C && r) {
        std::ofstream fs;
        fs.open("points.txt");
        auto& cld = _window->sensors_data().cloud();
        cld.mutex().lock();
        for(const point_t& p : cld.value().points()) {
            fs << std::fixed << p.x << "," << p.y << "," << p.z << std::endl;
        }
        cld.mutex().unlock();
        std::cout << "Point cloud was dumped to points.txt" << std::endl;
    }
    else if(k == GLFW_KEY_K && r) {
        auto& cld = _window->sensors_data().cloud();
        std::lock_guard<std::mutex> lock(cld.mutex());
        cld.value().clear();
        _renderer.clear_points();
    }
    else if(k == GLFW_KEY_P && r) {
        _window->sensors_data().pause = !_window->sensors_data().pause;
    }
}
