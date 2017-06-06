#include "view_debug.h"

#include <cstring>
#include <iostream>
#include <mutex>

#include <glm/gtc/matrix_transform.hpp>

#include "view_window.h"

extern const char* SHADER_2D_VS;
extern const char* SHADER_2D_FS;

view_debug::view_debug(view_window *parent) :
    view(parent),
    _update_time(0.0f)
{
    _2d_shader = std::shared_ptr<shader>(new shader(SHADER_2D_FS, SHADER_2D_VS));
    _font = std::shared_ptr<font>(new font("data/font.png", _2d_shader, 2.0f, 8, 8));
    _font->color = glm::vec3(1.0f, 1.0f, 1.0f);

    _proj = glm::ortho(0.0f, (float) parent->width(), (float) parent->height(), 0.0f);

    float data[] = {
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 0.0f,
        1.0f, 0.0f, 1.0f, 1.0f
    };

    _quad_buf = std::shared_ptr<vbuffer>(new vbuffer(data, 16, GL_TRIANGLE_STRIP));
    _quad_buf->setup_attrib(_2d_shader->get_attrib_location("position"), 2, 0, 4);
    _quad_buf->setup_attrib(_2d_shader->get_attrib_location("uvp"), 2, 2, 4);

    setup_lidar();
    setup_cameras();
}

view_debug::~view_debug()
{
}

void view_debug::update(float dt)
{
    _update_time += dt;

    if(_update_time > 1) {
        {
            auto& laser_res = _window->sensors_data().laser_img();
            std::lock_guard<std::mutex>(laser_res.mutex());
            image<float> img(laser_res.value().buf, laser_res.value().WIDTH, laser_res.value().HEIGHT, 1);
            _lidar_tex->bind();
            _lidar_tex->update(&img);
        }
        {
            auto& scam_res = _window->sensors_data().scam_img();
            std::lock_guard<std::mutex>(scam_res.mutex());
            image<> left(scam_res.value()[0].buf, scam_res.value()[0].WIDTH, scam_res.value()[0].HEIGHT, 1);
            image<> right(scam_res.value()[1].buf, scam_res.value()[1].WIDTH, scam_res.value()[1].HEIGHT, 1);
            _cam_left_tex->bind();
            _cam_left_tex->update(&left);
            _cam_right_tex->bind();
            _cam_right_tex->update(&right);
        }
        _update_time = 0.0f;
    }
}

void view_debug::draw()
{
    static unsigned char text_buf[512];

    _2d_shader->bind();
    _2d_shader->uniform_mat4("proj", _proj);
    _2d_shader->uniform3f("color", glm::vec3(1.0f, 1.0f, 1.0f));

    _quad_buf->bind();
    glm::mat4 scaled = glm::scale(glm::mat4(1.0f), glm::vec3(256.0f, 256.0f, 0.0f));
    glm::mat4 ident = glm::mat4(1.0f);

    _2d_shader->uniform1i("tex", 0);

    _lidar_tex->bind();
    _2d_shader->uniform_mat4("model", glm::translate(ident, glm::vec3(200, 20, 0)) * scaled);
    _quad_buf->draw(0, 4);

    _cam_left_tex->bind();
    _2d_shader->uniform_mat4("model", glm::translate(ident, glm::vec3(200, 320, 0)) * scaled);
    _quad_buf->draw(0, 4);

    _cam_right_tex->bind();
    _2d_shader->uniform_mat4("model", glm::translate(ident, glm::vec3(200 + 256 + 20, 320, 0)) * scaled);
    _quad_buf->draw(0, 4);

    snprintf((char*) text_buf, 511, "Lidar depth (%lu x %lu):", _lidar_tex->width(), _lidar_tex->height());
    _font->draw(text_buf, 200, 10);

    snprintf((char*) text_buf, 511, "Cam left (%lu x %lu):", _cam_left_tex->width(), _cam_left_tex->height());
    _font->draw(text_buf, 200, 310);

    snprintf((char*) text_buf, 511, "Cam right (%lu x %lu):", _cam_right_tex->width(), _cam_right_tex->height());
    _font->draw(text_buf, 200 + 256 + 20, 310);

    const char* connected = _window->sensors_data().client().is_connected() ? "connected" : "disconnected";
    snprintf((char*) text_buf, 511, "RAW\n"
                                    "%s\n\n"
                                    "IPC tick = %.2f sec\n",
                                     connected, _window->sensors_data().update_time());
    _font->draw(text_buf, 10, 10);
}

void view_debug::resize(int w, int h)
{
    _proj = glm::ortho(0.0f, (float) w, (float) h, 0.0f);
}

void view_debug::setup_lidar()
{
    const size_t W = vrep_client::image_msr_scam_t::WIDTH;
    const size_t H = vrep_client::image_msr_scam_t::HEIGHT;

    image<float> img(new float[W * H], W, H, 1, true);
    for(size_t x = 0; x < W * H; x++) img.data()[x] = 1.0f;

    _lidar_tex = std::shared_ptr<texture<float>>(texture<float>::from_image(&img));

    GLint swizzleMask[] = {GL_RED, GL_RED, GL_RED, GL_ONE};
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
}

void view_debug::setup_cameras()
{
    const size_t W = vrep_client::image_msr_scam_t::WIDTH;
    const size_t H = vrep_client::image_msr_scam_t::HEIGHT;

    image<> tmp(new uint8_t[W * H], W, H, 1, true);
    memset(tmp.data(), 0xff, W * H);

    GLint swizzleMask[] = {GL_RED, GL_RED, GL_RED, GL_ONE};

    _cam_left_tex = std::shared_ptr<texture<>>(texture<>::from_image(&tmp));
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
    _cam_right_tex = std::shared_ptr<texture<>>(texture<>::from_image(&tmp));
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
}
