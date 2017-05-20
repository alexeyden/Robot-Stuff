#include "main_view.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

#include <glm/gtc/matrix_transform.hpp>

extern const char* SHADER_2D_VS;
extern const char* SHADER_2D_FS;

main_view::main_view(view_window *parent) :
    view(parent), _renderer(*parent)
{
    _2d_shader = std::shared_ptr<shader>(new shader(SHADER_2D_FS, SHADER_2D_VS));
    _font = std::shared_ptr<font>(new font("data/font.png", _2d_shader, 2.0f, 8, 8));
    _font->color = glm::vec3(1.0f, 1.0f, 1.0f);

    _font_proj = glm::ortho(0.0f, (float) parent->width(), (float) parent->height(), 0.0f);

    _time = 0.0f;

    _pos = glm::vec3(-4.0f, 11.0f, 2.0f);
    _dir = glm::vec3(0.0f, 1.0f, 0.0f);
    _up = glm::vec3(0.0f, 0.0f, 1.0f);

    _angle_h = 0.0f;
    _angle_v = 0.0f;

    _cursor = true;
    _light = true;

    float data[] = {
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 0.0f,
        1.0f, 0.0f, 1.0f, 1.0f
    };

    _quad_buf = std::shared_ptr<vbuffer>(new vbuffer(data, 16, GL_TRIANGLE_STRIP));
    _quad_buf->setup_attrib(_2d_shader->get_attrib_location("position"), 2, 0, 4);
    _quad_buf->setup_attrib(_2d_shader->get_attrib_location("uvp"), 2, 2, 4);
}

void main_view::update(float dt)
{
    _time += dt;

    if(_time > 2.0f && _renderer.upload_points(_window->sensors_data())) {
        _time = 0.0f;
    }

    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_W) == GLFW_PRESS) {
        _pos += _dir * dt * 10.0f;
    }
    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_S) == GLFW_PRESS) {
        _pos -= _dir * dt * 10.0f;
    }
    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_A) == GLFW_PRESS) {
        _pos += glm::normalize(glm::cross(_up, _dir)) * dt * 5.0f;
    }
    if(glfwGetKey(_window->glfw_window(), GLFW_KEY_D) == GLFW_PRESS) {
        _pos -= glm::normalize(glm::cross(_up, _dir)) * dt * 5.0f;
    }

    if(_mesh_builder.result().valid() &&
            _mesh_builder.result().wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        auto res = _mesh_builder.result().get();
        _renderer.upload_mesh(*res);
        _mesh_builder.reset();
    }

    _renderer.update(dt);
}

void main_view::draw()
{
    static char buf[512];

    _renderer.view = glm::lookAt(_pos, _pos + _dir, _up);
    _renderer.eye = _pos;
    _renderer.dir = _dir;
    _renderer.light = _light;
    _renderer.render();

    _2d_shader->bind();
    _2d_shader->uniform_mat4("proj", _font_proj);

    bool mesh_job = _mesh_builder.result().valid() &&
            _mesh_builder.result().wait_for(std::chrono::seconds(0)) == std::future_status::timeout;

     snprintf(buf, 511, "pos = %.2f %.2f %.2f\n"
                        "a = %.1f b = %.1f\n"
                        "---------------------------\n"
                        "fps = %.1f\n"
                        "---------------------------\n"
                        "pts_n = %lu\n"
                        "trg_n = %lu\n"
                        "---------------------------\n"
                        "thresh = %.1f\n"
                        "---------------------------\n"
                        "mesh job = %d\n"
                        "---------------------------\n"
                        "pause = %d",
             _pos.x, _pos.y, _pos.z,
              glm::degrees(_angle_h), glm::degrees(_angle_v),
             _window->fps(),
             _renderer.points_count(),
             _renderer.vertex_count() / 3,
             _renderer.thresh,
             mesh_job,
             _window->sensors_data().pause);

    if(_renderer.points_mode)
        _font->draw((uint8_t*) "CLOUD", 10, 10);
    else
        _font->draw((uint8_t*) "MESH", 10, 10);

    _font->draw((uint8_t*) buf, 10, 40);

    _font->draw((uint8_t*) "l - lights\n"
                           "; - shadow\n"
                           "o - points\\mesh\n"
                           "e - ground plane\n"
                           "---------------------------\n"
                           "c - dump\n"
                           "k - nuke\n"
                           "---------------------------\n"
                           "v - build mesh (greedy)\n"
                           "b - build mesh (poisson)\n"
                           "n - build mesh (grid proj)\n"
                           "---------------------------\n"
                           "t - p thr. cut off\n"
                           "---------------------------\n"
                           "p - pause", 10, 200);

    if(!_cursor) {
        _font->draw((uint8_t*) "\x07", _window->width()/2 - 4, _window->height()/2 - 4);
    }

    /*
    glDisable(GL_DEPTH_TEST);
    _quad_buf->bind();
    glm::mat4 scaled = glm::scale(glm::mat4(1.0f), glm::vec3(200.0f, 200.0f, 0.0f));
    glm::mat4 ident = glm::mat4(1.0f);

    _2d_shader->uniform1i("tex", 0);

    glBindTexture(GL_TEXTURE_2D, _renderer.shadow_tex());
    _2d_shader->uniform_mat4("model", glm::translate(ident, glm::vec3(20, 20, 0)) * scaled);
    _quad_buf->draw(0, 4);
    glEnable(GL_DEPTH_TEST);
    */
}

void main_view::on_mouse(float x, float y)
{
    if(_mouse.x != 0.0f && _mouse.y != 0.0f && !_cursor) {
        float dx = x - _mouse.x;
        float dy = y - _mouse.y;

        _angle_v += dy * 0.01;
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
        _light ^= true;
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
        _window->sensors_data().pause ^= true;
    }
    else if(k == GLFW_KEY_T && r) {
        _renderer.thresh = (_renderer.thresh == 0.6f) ? 0.0f : 0.6f;
    }
    else if(k == GLFW_KEY_O && r) {
        _renderer.points_mode ^= true;
    }
    else if(k == GLFW_KEY_V && r) {
        _mesh_builder.build(_window->sensors_data().cloud(), mesh_builder::method::GREEDY);
    }
    else if(k == GLFW_KEY_B && r) {
        _mesh_builder.build(_window->sensors_data().cloud(), mesh_builder::method::POISSON);
    }
    else if(k == GLFW_KEY_N && r) {
        _mesh_builder.loc = _pos;
        _mesh_builder.build(_window->sensors_data().cloud(), mesh_builder::method::GRID_PROJ);
    }
    else if(k == GLFW_KEY_E && r) {
        _renderer.ground_plane ^= true;
    }
    else if(k == GLFW_KEY_SEMICOLON && r) {
        _renderer.shadow ^= true;
    }
}

void main_view::resize(int w, int h) {
  _font_proj = glm::ortho(0.0f, (float) w, (float) h, 0.0f);
  _renderer.resize();
}
