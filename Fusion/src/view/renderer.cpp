#include "renderer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/random.hpp>

static const char* vert_shadow_src = "#version 150\n"
                                     "in vec3 position;"
                                     "in vec2 uv;"
                                     "in vec3 norm;"
                                     "uniform mat4 proj;"
                                     "void main() {"
                                     "gl_Position = proj * vec4(position, 1.0);"
                                     "}";

static const char* frag_dummy_src = "#version 150\n"
                                     "out vec4 outColor;"
                                     "void main() {"
                                     "outColor = vec4(1.0);"
                                     "}";

extern const char* SHADER_3D_FS;
extern const char* SHADER_3D_VS;
extern const char* SHADER_3D_TEX_FS;
extern const char* SHADER_3D_TEX_VS;
extern const char* SHADER_CLOUD_FS;
extern const char* SHADER_CLOUD_VS;

renderer::renderer(const view_window &window) :
    _window(window)
{
    _shader = std::shared_ptr<shader>(new shader(SHADER_3D_FS, SHADER_3D_VS));
    _point_shader = std::shared_ptr<shader>(new shader(SHADER_CLOUD_FS, SHADER_CLOUD_VS));
    _shadow_shader = std::shared_ptr<shader>(new shader(frag_dummy_src, vert_shadow_src));

    view = glm::mat4();
    _proj = glm::perspective(glm::radians(45.0f), _window.width() / (float) _window.height(), 0.1f, 100.0f);

    float plane[] = {
        -50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,
        -50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,
         50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,
         50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f
    };

    _shader->bind();
    _plane = std::shared_ptr<vbuffer>(new vbuffer(plane, 4 * 7, GL_TRIANGLE_STRIP));
    _plane->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 7);
    _plane->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 7);
    _plane->setup_attrib(_shader->get_attrib_location("cluster"), 1, 6, 7);
    _plane->unbind();

    _mesh = std::shared_ptr<vbuffer>(new vbuffer(nullptr, 0, GL_TRIANGLES));
    _mesh->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 7);
    _mesh->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 7);
    _mesh->setup_attrib(_shader->get_attrib_location("cluster"), 1, 6, 7);
    _mesh->unbind();

    float cube[] = {
        //bottom
         0.0f,  1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.8f,
         0.0f,  0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.8f,
         1.0f,  1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.8f,
         0.0f,  0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.8f,
         1.0f,  1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.8f,
         1.0f,  0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.8f,

        //top
         0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.8f,
         0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.8f,
         1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.8f,
         0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.8f,
         1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.8f,
         1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.8f,

        //left
        0.0f,  1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.8f,
        0.0f,  0.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.8f,
        0.0f,  1.0f,  0.0f, -1.0f, 0.0f, 0.0f, 0.8f,
        0.0f,  0.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.8f,
        0.0f,  1.0f,  0.0f, -1.0f, 0.0f, 0.0f, 0.8f,
        0.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, 0.8f,

        //right
        1.0f,  1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.8f,
        1.0f,  0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.8f,
        1.0f,  1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.8f,
        1.0f,  0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.8f,
        1.0f,  1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.8f,
        1.0f,  0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.8f,

        //front
         0.0f, 1.0f,  1.0f, 0.0f, -1.0f, 0.0f, 0.8f,
         0.0f, 1.0f,  0.0f, 0.0f, -1.0f, 0.0f, 0.8f,
         1.0f, 1.0f,  1.0f, 0.0f, -1.0f, 0.0f, 0.8f,
         0.0f, 1.0f,  0.0f, 0.0f, -1.0f, 0.0f, 0.8f,
         1.0f, 1.0f,  1.0f, 0.0f, -1.0f, 0.0f, 0.8f,
         1.0f, 1.0f,  0.0f, 0.0f, -1.0f, 0.0f, 0.8f,

        //back
         0.0f, 0.0f,  1.0f, 0.0f, 1.0f, 0.0f, 0.8f,
         0.0f, 0.0f,  0.0f, 0.0f, 1.0f, 0.0f, 0.8f,
         1.0f, 0.0f,  1.0f, 0.0f, 1.0f, 0.0f, 0.8f,
         0.0f, 0.0f,  0.0f, 0.0f, 1.0f, 0.0f, 0.8f,
         1.0f, 0.0f,  1.0f, 0.0f, 1.0f, 0.0f, 0.8f,
         1.0f, 0.0f,  0.0f, 0.0f, 1.0f, 0.0f, 0.8f
    };

    _obj_mesh = std::shared_ptr<vbuffer>(new vbuffer(cube, sizeof(cube)/sizeof(float), GL_TRIANGLES));
    _obj_mesh->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 7);
    _obj_mesh->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 7);
    _obj_mesh->setup_attrib(_shader->get_attrib_location("cluster"), 1, 6, 7);
    _obj_mesh->unbind();

    init_shadows();

    float tmp[] = { 0.0f, 0.0f, 1.0f, 0.0f };
    _point_shader->bind();
    _points = std::shared_ptr<vbuffer>(new vbuffer(tmp, 4, GL_POINTS));
    _points->setup_attrib(_point_shader->get_attrib_location("position"), 3, 0, 4);
    _points->setup_attrib(_point_shader->get_attrib_location("prob"), 1, 3, 4);
    _points->unbind();

    _points_n = 1;
    _verts_n = 0;

    light_pos = glm::vec3(20.0f, 20.f, 20.0f);
    light_target = glm::vec3(0.0f, 8.0f, 0.0f);

    thresh = 0.0f;
}

void renderer::update(float dt)
{
    _t += dt;
}

void renderer::render()
{
    glm::vec3 lp0 = light_pos;
    glm::vec3 lp1 = light_target;
    glm::vec3 dir =  glm::normalize(lp1 - lp0);
    glm::vec3 right = glm::normalize(glm::cross(dir, glm::vec3(0.0f, 0.0f, 1.0f)));
    glm::mat4 model =
            glm::perspective(glm::radians(45.0f), _window.width() / (float) _window.height(), 0.1f, 100.0f) *
            glm::lookAt(lp0,  lp0 + dir, glm::normalize(glm::cross(right, dir)));

    bool use_shadow = shadow;
    if (use_shadow) {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _fbo);
        glClear(GL_DEPTH_BUFFER_BIT);

        _shadow_shader->bind();
        _shadow_shader->uniform_mat4("proj", model);

        _plane->bind();
        _plane->draw(0, 4);

        _mesh->bind();
        _mesh->draw(0, _verts_n);

        _obj_mesh->bind();
        _shadow_shader->uniform_mat4("proj", model * _robot);
        _obj_mesh->draw(0, 6 * 6);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    if(points_mode) {
        _point_shader->bind();
        _point_shader->uniform_mat4("proj", _proj);
        _point_shader->uniform_mat4("model", view);
        _point_shader->uniform1f("thresh", thresh);

        _points->bind();
        _points->draw(0, _points_n);
        _points->unbind();
    }

    _shader->bind();
    _shader->uniform_mat4("proj", _proj);
    _shader->uniform_mat4("model", view);
    _shader->uniform3f("eye", eye);
    _shader->uniform1i("light", light);
    _shader->uniform3f("light_dir", light_target - light_pos);
    _shader->uniform1i("shadow", use_shadow);

    if(use_shadow) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _shadow_map);
        _shader->uniform1i("map", 0);
        _shader->uniform_mat4("light_model", model);
    }

    if(!points_mode) {
        _shader->uniform3f("sva", glm::vec3(0.7f, 1.0f, 1.0f));
        _mesh->bind();
        _mesh->draw(0, _verts_n);
    }

    if(ground_plane) {
        _shader->uniform3f("sva", glm::vec3(0.0f, 1.0f, 1.0f));
        _plane->bind();
        _plane->draw(0, 4);
        _plane->unbind();
    }

    _obj_mesh->bind();
    _shader->uniform3f("sva", glm::vec3(0.9f, 1.0f, 0.5f));
    for(const glm::mat4& o : _objects) {
        _shader->uniform_mat4("model", view * o);
        _obj_mesh->draw(0, 6 * 6);
    }

    _shader->uniform4f("override_color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    _shader->uniform_mat4("model", view * _robot);
    _obj_mesh->draw(0, 6 * 6);
    _shader->uniform4f("override_color", glm::vec4(0.0f, 0.0f, 0.0f, 0.0f));

/*
    _xxx_shader->bind();
    _xxx_shader->uniform_mat4("proj", _proj);
    _xxx_shader->uniform_mat4("model", view);
    _xxx_shader->uniform3f("eye", eye);
    _xxx_shader->uniform1i("light", light);
    _xxx_shader->uniform3f("light_dir", light_target - light_pos);
    _xxx_shader->uniform1i("shadow", use_shadow);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _xxx_tex->id());
    _xxx_shader->uniform1i("tex", 1);

    if(use_shadow) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _shadow_map);
        _xxx_shader->uniform1i("map", 0);
        _xxx_shader->uniform_mat4("light_model", model);
    }

    _xxx_mesh->bind();
    for(const xxx::xxxp& xp : _xxx.xxxps()) {
        _xxx_shader->uniform_mat4("model", view * xp.model);
        _xxx_mesh->draw();
    }
    */

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

bool renderer::upload_points(sensors& fetcher)
{
    sensors::cloud_res_t& res = fetcher.cloud();

    if(!res.mutex().try_lock())
        return false;
    size_t to_load = res.value().points().size();

    if(to_load == 0) {
        res.mutex().unlock();
        return true;
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
    return true;
}

void renderer::upload_mesh(const std::vector<float> &data)
{
    _verts_n = data.size() / 7;

    _mesh->bind();
    _mesh->update(data.data(), data.size());
}

void renderer::update_objects(sensors &sensors)
{
    sensors.objects().mutex().lock();
    _objects.clear();
    for(const scam_object& o : sensors.objects().value()) {
        glm::mat4 m =
                glm::translate(glm::mat4(), o.pos) *
                glm::rotate(glm::mat4(), std::atan2(o.dir.y, o.dir.x), glm::vec3(0, 0, 1)) *
                glm::scale(glm::mat4(), o.size) *
                glm::translate(glm::mat4(), glm::vec3(-0.5f, -0.5f, -0.5f));
        _objects.push_back(m);
    }
    sensors.objects().mutex().unlock();

    auto pos_dir = *sensors.robot.load();
    auto pos = pos_dir.first;
    auto dir = pos_dir.second;
    _robot =  glm::translate(glm::mat4(), pos) *
                glm::rotate(glm::mat4(), std::atan2(dir.y, dir.x), glm::vec3(0, 0, 1)) *
                glm::scale(glm::mat4(), glm::vec3(0.65f, 0.38f, 0.42f)) *
                glm::translate(glm::mat4(), glm::vec3(-0.5f, -0.5f, -0.5f));

}

void renderer::clear_points()
{
    _points->bind();
    _points->update(nullptr, 0);
}

void renderer::init_shadows()
{
    glGenFramebuffers(1, &_fbo);

    glGenTextures(1, &_shadow_map);
    glBindTexture(GL_TEXTURE_2D, _shadow_map);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                 _window.width(), _window.height(), 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindFramebuffer(GL_FRAMEBUFFER, _fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _shadow_map, 0);

    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (Status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", Status);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void renderer::resize() {
}
